"""The calibration node (01-architecture §5a, §6-9): manual mode (M3).

    ros2 run handeye_autopilot calibrate_node --ros-args --params-file <params.yaml>

ROS 2 concepts used here (each links the official docs):
- an *action* server for the long-running calibration, with feedback and
  cancel (https://docs.ros.org/en/jazzy/Concepts/Basic/About-Actions.html);
- *services* for the manual capture trigger, finish and fault clearing
  (https://docs.ros.org/en/jazzy/Concepts/Basic/About-Services.html);
- a *MultiThreadedExecutor* with *callback groups*, so the run (which waits for
  TF and images) never blocks the subscriptions that deliver them
  (https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html);
- *QoS*: images use the sensor-data profile, /tf_static is transient-local
  (https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

The run executes in the action's execute callback on an executor worker
thread and waits only on events, TF lookups and images with deadlines; it
never spins. All calibration logic is in ``handeye_autopilot.core``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import UTC, datetime
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration as RclDuration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import tf2_ros
from visualization_msgs.msg import MarkerArray
import yaml

from handeye_autopilot_interfaces.action import Calibrate
from handeye_autopilot_interfaces.msg import GateResult, QualityReport

from ..core import codes, observations
from ..core import transforms as tf
from ..core.camera import CameraModel
from ..core.capture import CaptureRecord, Frame, capture
from ..core.quality import Calibration, calibrate
from ..core.results import result_dict, transform_dict
from ..core.solver import TOO_FEW_CAPTURES
from . import params as P
from . import tf_tools, viz
from .camera_input import CameraInfoError, ImageBuffer, camera_model, same_info
from .publish_result import PUBLISHABLE

R = Calibrate.Result
FB = Calibrate.Feedback
MODES = (Calibrate.Goal.MODE_EYE_TO_HAND, Calibrate.Goal.MODE_EYE_IN_HAND)
EXECUTIONS = (Calibrate.Goal.EXECUTION_MOVEIT, Calibrate.Goal.EXECUTION_MANUAL)


class RunAbort(Exception):
    """Ends a run as ABORTED (or CANCELED) with a result code and message."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code


@dataclass
class RunState:
    phase: str = FB.PHASE_PREFLIGHT
    accepted: int = 0
    rejected: int = 0
    last_rejection: str = ''
    last_detail: str = ''
    capture_request: threading.Event = field(default_factory=threading.Event)
    finish_request: threading.Event = field(default_factory=threading.Event)
    cancelled: threading.Event = field(default_factory=threading.Event)
    capturing: bool = False
    min_captures: int = 0
    target_captures: int = 0
    snapshot: dict[str, Any] = field(default_factory=dict)   # parameters at acceptance
    profile_text: str | None = None     # the profile file as read at acceptance
    info: CameraInfo | None = None      # the run's CameraInfo (fixed after preflight)
    info_changed: bool = False          # any differing CameraInfo arrived since
    jumps: int = 0                      # clock-jump count at acceptance
    finalizing: bool = False            # past the point where a cancel can win
    max_now: float = 0.0                # latest node time seen (sampled jump check)
    records: list[CaptureRecord] = field(default_factory=list)


class CalibrationNode(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('handeye_autopilot', **kwargs)
        P.declare(self)
        self._sensors = ReentrantCallbackGroup()
        self._server = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._run: RunState | None = None
        self._fault_latched = False

        # camera input (§7)
        self.images = ImageBuffer(
            self.now, wall_clock=lambda: not self.get_parameter('use_sim_time').value)
        self._info: CameraInfo | None = None
        self._info_lock = threading.Lock()
        self.create_subscription(Image, 'image', self._on_image, qos_profile_sensor_data,
                                 callback_group=self._sensors)
        # best-effort + volatile is compatible with every publisher (reliable or not,
        # latched or not); CameraInfo is re-sent with each image by camera drivers
        self.create_subscription(CameraInfo, 'camera_info', self._on_info,
                                 qos_profile_sensor_data, callback_group=self._sensors)

        # TF (§8-9)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.static_links = tf_tools.StaticLinks()
        static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, '/tf_static', self.static_links.add, static_qos,
                                 callback_group=self._sensors)
        self.dynamic_links = tf_tools.StaticLinks()          # edges seen on /tf
        # best-effort is compatible with every publisher, so a best-effort bridge
        # publishing a competing parent is seen too
        self.create_subscription(TFMessage, '/tf', self.dynamic_links.add,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
                                 callback_group=self._sensors)
        self._stopping = threading.Event()
        # backwards clock jumps (sim time reset): counted by rclpy's jump callback,
        # so a reset between two samples is never missed; the TF and image history
        # of the old epoch is dropped at once
        self._jumps = 0
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=RclDuration(nanoseconds=-1),
                          on_clock_change=True), post_callback=self._on_jump)
        self.last_run_records: list[CaptureRecord] = []
        self.last_result: Calibrate.Result | None = None
        self._solve_end: Any = None          # node time the last final solve finished
        self._joints: list[tuple[float, JointState]] = []
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10,
                                 callback_group=self._sensors)

        # outputs
        self.pub_image = self.create_publisher(Image, '~/annotated_image', 2)
        self.pub_markers = self.create_publisher(MarkerArray, '~/markers', 2)

        # action + services
        self._action = ActionServer(
            self, Calibrate, '~/calibrate', execute_callback=self._execute,
            goal_callback=self._admit, cancel_callback=self._on_cancel,
            callback_group=self._server)
        self.create_service(Trigger, '~/capture', self._srv_capture,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, '~/finish', self._srv_finish,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(Trigger, '~/clear_fault', self._srv_clear_fault,
                            callback_group=self._server)
        if self.get_parameter('testing.allow_fault_injection').value:
            self.get_logger().warning('testing.allow_fault_injection is ON: ~/inject_fault '
                                      'can latch a motion fault (tests only)')
            self.create_service(Trigger, '~/inject_fault', self._srv_inject_fault,
                                callback_group=self._server)
        self.get_logger().info('ready: send a goal to ~/calibrate')

    # -- small helpers ------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_jump(self, jump: Any) -> None:
        with self._lock:
            self._jumps += 1
        self.tf_buffer.clear()              # static transforms are kept by tf2
        self.images.clear()
        self.get_logger().warning('the clock jumped backwards: TF and image history cleared')

    def _on_info(self, msg: CameraInfo) -> None:
        with self._info_lock:
            self._info = msg
        with self._lock:
            run = self._run
            if run is not None and run.info is not None and not same_info(run.info, msg):
                run.info_changed = True     # even a brief change taints the run (§7)

    def _on_image(self, msg: Image, msg_info: Any = None) -> None:
        try:
            self.images.add(msg, msg_info)
        except Exception as e:  # noqa: BLE001 - a bad stream must not kill the node
            self.get_logger().warning(f'cannot use image ({msg.encoding}): {e}',
                                      throttle_duration_sec=5.0)

    def _on_joints(self, msg: JointState) -> None:
        with self._lock:
            self._joints.append((time.monotonic(), msg))
            self._joints = self._joints[-200:]

    # -- admission (§5): rejected goals get no result -----------------------------
    def _admit(self, goal: Calibrate.Goal) -> GoalResponse:
        reason = ''
        with self._lock:
            if self._run is not None:
                reason = 'a calibration run is already active'
            elif self._fault_latched:
                reason = 'motion fault latched: check the robot, then call ~/clear_fault'
            elif goal.mode not in MODES:
                reason = f'mode {goal.mode!r} is not one of {MODES}'
            elif goal.execution not in EXECUTIONS:
                reason = f'execution {goal.execution!r} is not one of {EXECUTIONS}'
            profile_text = None
            if not reason and goal.profile:
                path = P.profile_path(self, goal.profile)
                if path is None:
                    reason = f'profile {goal.profile!r} not found in profiles_dir'
                else:
                    try:
                        profile_text = path.read_text()      # read once, here
                    except (OSError, UnicodeDecodeError) as e:
                        reason = f'profile {goal.profile!r} cannot be read: {e}'
            if not reason:
                # reserve the slot before the next goal, and snapshot the parameters
                # now: later parameter changes must not reach an accepted run (§5)
                self._run = RunState(snapshot=P.node_values(self), jumps=self._jumps,
                                     profile_text=profile_text)
        if reason:
            self.get_logger().warning(f'goal rejected: {reason}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: Any) -> CancelResponse:
        """Accept (rclpy's default rejects), unless the run is already finalizing:
        then the outcome is decided and a cancel could not be honoured."""
        with self._lock:
            if self._run is not None:
                if self._run.finalizing:
                    return CancelResponse.REJECT
                self._run.cancelled.set()
        return CancelResponse.ACCEPT

    # -- services -------------------------------------------------------------------
    def _srv_capture(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        with self._lock:
            run = self._run
            if run is None or run.phase != FB.PHASE_MANUAL_CAPTURE:
                res.success, res.message = False, 'not in the manual capture phase'
            elif run.capturing or run.capture_request.is_set():
                res.success, res.message = False, 'a capture is already running'
            elif run.finish_request.is_set() or run.accepted >= run.target_captures:
                res.success, res.message = False, 'the run is finishing (solving next)'
            else:
                run.capture_request.set()
                res.success, res.message = True, 'capture started; the outcome arrives in feedback'
        return res

    def _srv_finish(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        with self._lock:
            run = self._run
            if run is None or run.phase not in (FB.PHASE_MANUAL_CAPTURE, FB.PHASE_CAPTURING):
                res.success, res.message = False, 'not in the manual capture phase'
            elif run.accepted < run.min_captures:
                res.success, res.message = False, (
                    f'{run.accepted} accepted captures; at least {run.min_captures} needed')
            else:
                run.finish_request.set()
                res.success, res.message = True, 'solving after the current capture'
        return res

    def _srv_inject_fault(self, req: Trigger.Request, res: Trigger.Response
                          ) -> Trigger.Response:
        with self._lock:
            self._fault_latched = True
        self.get_logger().error('motion fault latched (injected for testing)')
        res.success, res.message = True, 'fault latched'
        return res

    def _srv_clear_fault(self, req: Trigger.Request, res: Trigger.Response
                         ) -> Trigger.Response:
        """§6: clears only when no run is active and fresh joint_states (newer than
        the call) show the robot still over a window."""
        with self._lock:
            if not self._fault_latched:
                res.success, res.message = True, 'no fault latched'
                return res
            if self._run is not None:
                res.success, res.message = False, 'a run is active'
                return res
        t0, call_now = time.monotonic(), self.now()
        window, vmax, deadline, min_msgs = 0.5, 0.01, t0 + 3.0, 5
        waiting = ''
        while time.monotonic() < deadline:
            with self._lock:
                # fresh: arrived after the call AND stamped after it (not a replay)
                fresh = [(t, m) for t, m in self._joints if t > t0 and
                         m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= call_now - 0.1]
            stamps = sorted({m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                             for _, m in fresh})
            # at least 5 distinct measurements whose own stamps span the window
            if len(stamps) >= min_msgs and stamps[-1] - stamps[0] >= window:
                msgs = [m for _, m in fresh]
                if any(not np.all(np.isfinite(m.position)) or
                       not np.all(np.isfinite(m.velocity)) for m in msgs):
                    res.success, res.message = False, 'joint_states contain NaN or inf'
                    return res
                if any(m.velocity and max(abs(v) for v in m.velocity) > vmax for m in msgs):
                    res.success, res.message = False, 'joints are still moving'
                    return res
                series: dict[str, list[tuple[float, float]]] = {}  # per joint NAME (several
                for m in msgs:                                       # publishers may share it)
                    st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                    for name, p in zip(m.name, m.position, strict=False):
                        series.setdefault(name, []).append((st, p))
                if not series:
                    res.success, res.message = False, 'joint_states carry no positions'
                    return res
                # every joint the robot has reported recently (also before the
                # call) needs its own fresh evidence: 5 samples spanning the window
                with self._lock:
                    known = {n for _, m in self._joints for n in m.name}
                thin = [n for n in known | set(series) if n not in series or
                        len(series[n]) < min_msgs or
                        max(t for t, _ in series[n]) - min(t for t, _ in series[n]) < window]
                if thin:              # keep collecting until the deadline
                    waiting = f'not enough fresh samples for joint(s) {", ".join(sorted(thin))}'
                    time.sleep(0.05)
                    continue
                series = {n: [p for _, p in v] for n, v in series.items()}
                span = max(float(np.ptp(v)) for v in series.values())  # type: ignore[arg-type]
                if span > vmax * window:
                    res.success, res.message = False, 'joint positions are still changing'
                    return res
                with self._lock:
                    self._fault_latched = False
                self.get_logger().info('motion fault cleared')
                res.success, res.message = True, 'fault cleared'
                return res
            time.sleep(0.05)
        res.success, res.message = False, waiting or (
            'no fresh /joint_states (at least 5 messages over '
            f'{window} s, stamped after the call): cannot confirm the robot has stopped')
        return res

    # -- the run ------------------------------------------------------------------
    def _execute(self, goal_handle: Any) -> Calibrate.Result:
        run = self._run
        assert run is not None
        goal = goal_handle.request
        try:
            result = self._run_goal(goal_handle, goal, run)
            result = self._final_verdict(run, result)
            self._terminate(goal_handle, result)
            return result
        except Exception as e:  # noqa: BLE001 - any bug must still end the goal
            self.get_logger().error(f'run failed: {e!r}')
            result = self._final_verdict(run, self._result(R.INTERNAL,
                                                           f'internal error: {e!r}'))
            self._terminate(goal_handle, result)     # an accepted cancel is honoured here too
            return result
        finally:
            self.last_run_records = list(run.records)
            self.last_result = locals().get('result')       # for diagnostics and tests
            with self._lock:
                self._run = None

    def _terminate(self, goal_handle: Any, result: Calibrate.Result) -> None:
        """The one place a goal reaches its terminal state. Only a success leaves
        the result publishable: the marker is created after the final verdict."""
        if result.error_code == R.NONE and result.result_file:
            try:
                (Path(result.result_file).parent / PUBLISHABLE).touch()
            except OSError as e:
                self.get_logger().error(f'result kept but not publishable: {e}')
        if result.error_code == R.CANCELED:
            # the cancel callback runs before rclpy moves the goal to CANCELING:
            # wait for that transition so canceled() is legal
            t0 = time.monotonic()
            while not goal_handle.is_cancel_requested and time.monotonic() - t0 < 2.0:
                time.sleep(0.01)
        if result.error_code == R.CANCELED and goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif result.error_code == R.NONE:
            goal_handle.succeed()
        else:
            goal_handle.abort()

    def _final_verdict(self, run: RunState, result: Calibrate.Result) -> Calibrate.Result:
        """Decide the outcome once, under the lock: from here no cancel is accepted
        (the cancel callback checks ``finalizing``), and anything that happened up
        to this instant (cancel, clock jump, CameraInfo change) wins over success."""
        now = self.now()
        with self._lock:
            run.finalizing = True
            cancel = run.cancelled.is_set()
            jumped = self._jumps != run.jumps or now < run.max_now - 0.001
            info_changed = run.info_changed
        kept = f' (result file {result.result_file} was written first; marked '\
            'unpublishable)' if result.result_file else ''
        overridden = result.error_code == R.NONE and (cancel or jumped or info_changed)
        if overridden and result.result_file:
            self._mark_unpublishable(result.result_file,
                                     'cancelled' if cancel else 'clock jumped back' if jumped
                                     else 'CameraInfo changed')
        if cancel and result.error_code != R.CANCELED:
            return self._result(R.CANCELED, 'cancelled' + kept,
                                result_file=result.result_file)
        if result.error_code == R.NONE and jumped:
            return self._result(R.TF_UNAVAILABLE, 'the node clock jumped back during the '
                                'run; the result is not trusted' + kept,
                                result_file=result.result_file)
        if result.error_code == R.NONE and info_changed:
            return self._result(R.NO_CAMERA_INFO, 'CameraInfo changed during the run; the '
                                'result is not trusted' + kept, result_file=result.result_file)
        return result

    def _mark_unpublishable(self, path: str, why: str) -> None:
        try:
            doc = yaml.safe_load(Path(path).read_text()) or {}
            doc['publish'] = {'ok': False, 'reason': f'run invalidated after writing: {why}'}
            Path(path).write_text(yaml.safe_dump(doc, sort_keys=False))
        except (OSError, yaml.YAMLError) as e:
            self.get_logger().error(f'could not mark {path} unpublishable: {e}')

    def _feedback(self, goal_handle: Any, run: RunState, phase: str | None = None) -> None:
        if phase is not None:
            with self._lock:
                run.phase = phase
        fb = FB()
        fb.phase, fb.accepted, fb.rejected = run.phase, run.accepted, run.rejected
        fb.last_rejection, fb.last_rejection_detail = run.last_rejection, run.last_detail
        goal_handle.publish_feedback(fb)

    def _result(self, code: int, message: str, cal: Calibration | None = None,
                frames: dict[str, str] | None = None, result_file: str = '') -> Calibrate.Result:
        r = R()
        r.error_code, r.message, r.result_file = code, message, result_file
        r.quality = quality_msg(cal)
        if cal is not None and cal.solution is not None and frames is not None:
            r.transform_valid, r.passed = True, cal.passed
            stamp = (self._solve_end or self.get_clock().now()).to_msg()   # §5: solve end
            parent = frames['base_frame'] if cal.mode == MODES[0] else frames['tool_frame']
            zpar = frames['tool_frame'] if cal.mode == MODES[0] else frames['base_frame']
            r.camera_transform = tf_tools.to_msg(cal.solution.X, parent,
                                                 frames['camera_frame'], stamp)
            r.target_transform = tf_tools.to_msg(cal.solution.Z, zpar,
                                                 frames['board_frame'], stamp)
        else:
            r.camera_transform.transform.rotation.w = 1.0
            r.target_transform.transform.rotation.w = 1.0
        return r

    def _check_cancel(self, goal_handle: Any, run: RunState) -> None:
        if self._stopping.is_set():
            run.cancelled.set()
            raise RunAbort(R.INTERNAL, 'the node is shutting down')
        if goal_handle.is_cancel_requested or run.cancelled.is_set():
            run.cancelled.set()
            raise RunAbort(R.CANCELED, 'cancelled')

    def stop(self) -> None:
        """Shutdown: end any run promptly so the executor can stop (Ctrl-C)."""
        self._stopping.set()
        with self._lock:
            if self._run is not None:
                self._run.cancelled.set()

    def _run_goal(self, goal_handle: Any, goal: Calibrate.Goal, run: RunState
                  ) -> Calibrate.Result:
        self._solve_end = None
        try:
            cfg = P.load_run_config(self, goal.profile, run.snapshot, run.profile_text)
        except ValueError as e:
            return self._result(R.INVALID_GOAL, str(e))
        with self._lock:
            # ~/finish needs enough captures for every k-fold training split
            run.min_captures = P.least_captures(cfg.solver.min_captures, cfg.solver.folds)
            run.target_captures = cfg.session.target_captures
        try:
            camera, info = self._preflight(goal_handle, run, cfg)
            if goal.execution == Calibrate.Goal.EXECUTION_MOVEIT:
                raise RunAbort(R.INTERNAL, 'MoveIt execution is not available yet (M4); '
                               'use execution "manual"')
            records = self._manual(goal_handle, run, cfg, camera, info)
            self._feedback(goal_handle, run, FB.PHASE_SOLVING)
            frames = dict(cfg.frames, camera_frame=info.header.frame_id)
            t_solve = self.now()
            cal = calibrate(goal.mode, records, cfg.solver)
            self._solve_end = self.get_clock().now()
            # a cancel or a clock reset during the solve still wins (no result written)
            self._check_cancel(goal_handle, run)
            self._check_jump(t_solve)
        except RunAbort as e:
            return self._result(e.code, str(e))
        self._feedback(goal_handle, run, FB.PHASE_VERIFYING)
        path = self._write_results(goal.mode, cfg, records, cal, frames)
        if cal.solution is None:
            reason = cal.failure.reason if cal.failure else 'unknown'
            msg = f'solve failed ({reason}): {cal.failure}; observations saved for an ' \
                  'offline re-solve'
            # 02-features §E5: too_few_captures -> NOT_ENOUGH_SAMPLES, others SOLVE_FAILED
            code = R.NOT_ENOUGH_SAMPLES if reason == TOO_FEW_CAPTURES else R.SOLVE_FAILED
            return self._result(code, msg, cal, frames, str(path))
        msg = 'calibration passed all quality gates' if cal.passed else \
            'calibration finished but failed quality gates: ' + ', '.join(
                g.name for g in cal.report.gates if g.gated and not g.passed)
        return self._result(R.NONE, msg, cal, frames, str(path))

    # -- preflight (§7-8) -------------------------------------------------------------
    def _preflight(self, goal_handle: Any, run: RunState, cfg: P.RunConfig
                   ) -> tuple[CameraModel, CameraInfo]:
        self._feedback(goal_handle, run, FB.PHASE_PREFLIGHT)
        t = cfg.timing
        deadline = time.monotonic() + t.camera_info_timeout
        while True:
            self._check_cancel(goal_handle, run)
            with self._info_lock:
                info = self._info
            img = self.images.latest()
            if info is not None and img is not None:
                break
            if time.monotonic() > deadline:
                what = 'CameraInfo' if info is None else 'images'
                raise RunAbort(R.NO_CAMERA_INFO, f'no {what} received within '
                               f'{t.camera_info_timeout:.0f} s; check the camera topics '
                               '(remap "image" and "camera_info")')
            time.sleep(0.05)
        try:
            camera = camera_model(info, cfg.image_is_rectified)
        except CameraInfoError as e:
            raise RunAbort(R.NO_CAMERA_INFO, str(e)) from e
        with self._lock:
            run.info = info                  # pinned now: any later change taints the run
        if img.frame_id != info.header.frame_id:
            raise RunAbort(R.NO_CAMERA_INFO, f'image frame {img.frame_id!r} differs from '
                           f'CameraInfo frame {info.header.frame_id!r}')
        if img.gray.shape[1] != info.width or img.gray.shape[0] != info.height:
            raise RunAbort(R.NO_CAMERA_INFO, 'image size differs from CameraInfo')
        self._check_clock(img.stamp, img.received, cfg)
        base, tool = cfg.frames['base_frame'], cfg.frames['tool_frame']
        # TF at an image's stamp, retried with the NEWEST image each time (an early
        # frame can predate this node's TF history), until the deadline
        tf_deadline = time.monotonic() + max(t.tf_timeout, t.camera_info_timeout)
        found = None
        while found is None and time.monotonic() < tf_deadline:
            img = self.images.latest() or img
            self._check_clock(img.stamp, img.received, cfg)
            found = self._lookup_cancellable(goal_handle, run, base, tool, img.stamp_ns,
                                             min(0.2, t.tf_timeout))
        if found is None:
            if tf_tools.latest(self.tf_buffer, base, tool) is None:
                why = 'is the robot driver (robot_state_publisher) running?'
            else:
                why = ('TF exists but not at the image stamp: the robot and the camera may '
                       'use different clocks (use_sim_time / device clocks)')
            raise RunAbort(R.TF_UNAVAILABLE, f'no TF {base} -> {tool} at the image stamp; '
                           + why)
        return camera, info

    def _check_jump(self, since: float, run: RunState | None = None) -> None:
        """A backwards clock jump (sim time reset) aborts the run; the jump
        callback has already dropped the old epoch's TF and image history. A host
        clock step (wall time; no ROS jump callback) is caught by sampling."""
        now = self.now()
        run = run or self._run
        if run is not None:
            run.max_now = max(run.max_now, now)
        jumped = run is not None and (self._jumps != run.jumps or now < run.max_now - 0.001)
        if jumped or now < since - 0.5:
            self.tf_buffer.clear()          # static transforms are kept by tf2
            self.images.clear()
            raise RunAbort(R.TF_UNAVAILABLE, 'the node clock jumped back during the run '
                           '(sim time reset or a host clock step); TF and image history '
                           'are no longer comparable')

    def _check_clock(self, stamp: float, now: float, cfg: P.RunConfig) -> None:
        """§8 one clock domain: an image's stamp and the node clock when it
        arrived must agree (arrival time, so waiting on TF later cannot fake a
        clock mismatch)."""
        if abs(stamp - now) > cfg.timing.clock_tolerance:
            sim = self.get_parameter('use_sim_time').value
            raise RunAbort(R.TF_UNAVAILABLE, (
                f'image stamp {stamp:.3f} s and node clock {now:.3f} s differ by '
                f'{abs(stamp - now):.2f} s (> {cfg.timing.clock_tolerance} s): the camera '
                f'and the node use different clocks (node use_sim_time={sim}); stamp images '
                'with the host / ROS clock'))

    # -- manual capture loop -------------------------------------------------------
    def _check_info(self, run: RunState) -> None:
        if run.info_changed:
            raise RunAbort(R.NO_CAMERA_INFO, 'CameraInfo changed during the run; intrinsics '
                           'are fixed per run')

    def _manual(self, goal_handle: Any, run: RunState, cfg: P.RunConfig, camera: CameraModel,
                info: CameraInfo) -> list[CaptureRecord]:
        records: list[CaptureRecord] = []
        self._feedback(goal_handle, run, FB.PHASE_MANUAL_CAPTURE)
        last_now = self.now()
        while True:
            self._check_cancel(goal_handle, run)
            self._check_jump(last_now)
            last_now = max(last_now, self.now())
            self._check_info(run)
            # one atomic decision (the services take the same lock): a capture
            # acknowledged before ~/finish is always taken first
            with self._lock:
                take = run.capture_request.is_set()
                if take:
                    run.capturing = True
                    run.capture_request.clear()
                elif run.finish_request.is_set() or run.accepted >= cfg.session.target_captures:
                    return records
            if not take:
                run.capture_request.wait(0.05)
                continue
            if take:
                try:
                    self._feedback(goal_handle, run, FB.PHASE_CAPTURING)
                    rec = self._capture_once(goal_handle, run, cfg, camera, info, len(records))
                    records.append(rec)
                    run.records = records
                    with self._lock:
                        if rec.accepted:
                            run.accepted += 1
                            run.last_rejection, run.last_detail = '', ''
                        else:
                            run.rejected += 1
                            run.last_rejection = rec.reject or ''
                            run.last_detail = rec.detail
                finally:
                    with self._lock:
                        run.capturing = False
                self._feedback(goal_handle, run, FB.PHASE_MANUAL_CAPTURE)

    def _capture_once(self, goal_handle: Any, run: RunState, cfg: P.RunConfig,
                      camera: CameraModel, info: CameraInfo, cid: int) -> CaptureRecord:
        t = cfg.timing
        base, tool = cfg.frames['base_frame'], cfg.frames['tool_frame']
        t_start = self.now()
        s = tf_tools.settle(self.tf_buffer, base, tool, self.now, t.settle_window,
                            t.settle_translation, t.settle_rotation, t.settle_timeout,
                            run.cancelled)
        self._check_cancel(goal_handle, run)
        self._check_jump(t_start)
        if not s.ok:
            if 'newer than the settle start' in s.reason:
                raise RunAbort(R.TF_UNAVAILABLE, s.reason)
            return reject_record(cid, camera, codes.TF_UNAVAILABLE, s.reason)
        n = cfg.session.frames_per_capture
        # frames that ARRIVED after the settle: each is checked (camera, clock), and
        # only those also ACQUIRED after the settle are used (§8)
        images: list[Any] = []
        seen: set[int] = set()
        deadline = time.monotonic() + t.frame_timeout
        while len(images) < n and time.monotonic() < deadline:
            got = self.images.wait_received_after(s.end_stamp, n + len(seen), 0.1,
                                                  run.cancelled)
            self._check_cancel(goal_handle, run)
            for im in got:
                if im.stamp_ns in seen:
                    continue
                seen.add(im.stamp_ns)
                if im.frame_id != info.header.frame_id or \
                        im.gray.shape[:2] != (info.height, info.width):
                    raise RunAbort(R.NO_CAMERA_INFO, f'image from {im.frame_id!r} '
                                   f'({im.gray.shape[1]}x{im.gray.shape[0]}) does not match '
                                   f'the CameraInfo of {info.header.frame_id!r}')
                self._check_clock(im.stamp, im.received, cfg)
                if im.stamp > s.end_stamp and len(images) < n:
                    images.append(im)
        if len(images) < n:
            return reject_record(cid, camera, codes.NO_BOARD if not images else codes.CONSENSUS,
                                 f'{len(images)} of {n} images acquired after the settle within '
                                 f'{t.frame_timeout} s')
        frames: list[Frame] = []
        for im in images:
            pose = self._lookup_cancellable(goal_handle, run, base, tool, im.stamp_ns,
                                            t.tf_timeout)
            frames.append(Frame(im.gray, im.stamp, pose))
            self._check_cancel(goal_handle, run)
        rec = capture(cid, frames, cfg.board, camera, cfg.capture)
        self._check_info(run)               # intrinsics unchanged while these were taken
        shown = min(images, key=lambda im: abs(im.stamp - (rec.stamp or images[-1].stamp)))
        self._publish_viz(shown, rec, cfg, camera)
        return rec

    def _lookup_cancellable(self, goal_handle: Any, run: RunState, base: str, tool: str,
                            stamp_ns: int, timeout: float) -> Any:
        """TF at a stamp, waited for in 0.1 s slices so a cancel is never held up."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            pose = tf_tools.lookup_at(self.tf_buffer, base, tool, stamp_ns,
                                      max(0.0, min(0.1, left)))
            if pose is not None or left <= 0:
                return pose
            self._check_cancel(goal_handle, run)

    def _publish_viz(self, im: Any, rec: CaptureRecord, cfg: P.RunConfig,
                     camera: CameraModel) -> None:
        header = Header(frame_id=im.frame_id)
        header.stamp = tf_tools.stamp_to_time(im.stamp).to_msg()
        try:
            self.pub_image.publish(viz.annotated_image(im.gray, rec, cfg.board, header))
            self.pub_markers.publish(viz.board_markers(rec, cfg.board, im.frame_id,
                                                       header.stamp))
        except Exception as e:  # noqa: BLE001 - visualisation must never end a run
            self.get_logger().warning(f'visualisation failed: {e!r}')

    # -- results (§9) ------------------------------------------------------------------
    def _write_results(self, mode: str, cfg: P.RunConfig, records: list[CaptureRecord],
                       cal: Calibration, frames: dict[str, str]) -> Path:
        run_dir = Path(cfg.output_dir).expanduser() / datetime.now(UTC).strftime(
            '%Y%m%dT%H%M%S_%fZ')
        run_dir.mkdir(parents=True, exist_ok=True)
        meta = {'mode': mode, 'frames': frames, 'profile': cfg.profile}
        observations.save(run_dir / 'observations.yaml', records, cfg.board, meta)
        doc = result_dict(cal, cfg.solver, frames, meta)
        doc['publish'] = self._publish_section(mode, cal, frames, cfg)
        doc['config'] = _plain(cfg.snapshot)
        path = run_dir / 'result.yaml'
        path.write_text(yaml.safe_dump(doc, sort_keys=False))
        self.get_logger().info(f'result written to {path}')
        return path

    def _publish_section(self, mode: str, cal: Calibration, frames: dict[str, str],
                         cfg: P.RunConfig) -> dict[str, Any]:
        """§9 subtree rule + ownership checks: what result.launch.py may publish."""
        if cal.solution is None:
            return {'ok': False, 'reason': 'no transform (solve failed)'}
        parent = frames['base_frame'] if mode == MODES[0] else frames['tool_frame']
        optical = frames['camera_frame']
        tree = tf_tools.parents(self.tf_buffer)
        sub = tf_tools.camera_subtree(tree, self.static_links, optical, parent,
                                      cfg.frames.get('camera_root_frame', ''),
                                      self.dynamic_links)
        if sub.error:
            self.get_logger().warning(f'result not publishable: {sub.error}')
            return {'ok': False, 'reason': sub.error}
        if sub.root == optical:
            T_root_opt = np.eye(4)
        else:
            got = tf_tools.latest(self.tf_buffer, sub.root, optical)
            if got is None:
                return {'ok': False, 'reason': f'no TF {sub.root} -> {optical}'}
            T_root_opt = got[0]
        T_parent_root = cal.solution.X @ tf.inv(T_root_opt)
        return {'ok': True, 'reason': '',
                'transform': transform_dict(T_parent_root, parent, sub.root),
                # the camera subtree at calibration time: publish_result refuses if
                # the live one differs (a driver restart with other extrinsics)
                'subtree': transform_dict(T_root_opt, sub.root, optical)}


def reject_record(cid: int, camera: CameraModel, code: str, detail: str) -> CaptureRecord:
    return CaptureRecord(cid, None, camera, np.zeros(0, np.int32), None, None, None, None,
                         None, None, False, code, detail)


def quality_msg(cal: Calibration | None) -> QualityReport:
    """Core QualityReport -> message; uncomputed floats are NaN (§5)."""
    q = QualityReport()
    nan = math.nan
    for f in ('reprojection_rms_px', 'reprojection_max_px', 'relative_rotation_max_rad',
              'rotation_axis_spread', 'scaled_min_singular_value', 'fold_translation_max_m',
              'fold_rotation_max_rad', 'heldout_reprojection_rms_px',
              'heldout_reprojection_max_px'):
        setattr(q, f, nan)
    if cal is None:
        return q
    r = cal.report
    for f in ('captures_accepted', 'captures_rejected', 'inliers', 'outliers', 'folds',
              'folds_failed'):
        setattr(q, f, int(getattr(r, f)))
    for f in ('reprojection_rms_px', 'reprojection_max_px', 'relative_rotation_max_rad',
              'rotation_axis_spread', 'scaled_min_singular_value', 'fold_translation_max_m',
              'fold_rotation_max_rad', 'heldout_reprojection_rms_px',
              'heldout_reprojection_max_px'):
        setattr(q, f, float(getattr(r, f)))
    for g in r.gates:
        m = GateResult()
        m.name, m.group, m.unit, m.comparison = g.name, g.group, g.unit, g.comparison
        m.value, m.limit, m.gated, m.passed = float(g.value), float(g.limit), g.gated, g.passed
        q.gates.append(m)
    return q


def _plain(v: Any) -> Any:
    if isinstance(v, dict):
        return {k: _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    return v


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CalibrationNode()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()                 # an active run ends promptly, so shutdown can finish
        ex.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
