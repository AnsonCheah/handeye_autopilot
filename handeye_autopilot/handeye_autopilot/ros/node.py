"""The calibration node (01-architecture §5a, §6-9): manual mode (M3) and
MoveIt mode (M4).

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

from dataclasses import dataclass, field, replace
from datetime import UTC, datetime
import math
from pathlib import Path
import signal
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
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
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
import tf2_ros
from visualization_msgs.msg import MarkerArray
import yaml

from handeye_autopilot_interfaces.action import Calibrate
from handeye_autopilot_interfaces.msg import GateResult, QualityReport

from ..core import codes, observations
from ..core import session as S
from ..core import transforms as tf
from ..core.camera import CameraModel
from ..core.capture import CaptureRecord, Frame, capture
from ..core.planning import IKResult, box_contains, lever_ball_points
from ..core.quality import Calibration, calibrate
from ..core.results import result_dict, transform_dict
from ..core.solver import TOO_FEW_CAPTURES
from . import params as P
from . import tf_tools, viz
from .camera_input import CameraInfoError, ImageBuffer, camera_model, same_info
from .moveit_client import (
    INVALID_MOTION_PLAN,
    SUCCESS,
    TERMINAL,
    GoalOutcome,
    MoveItClient,
    dense_samples,
    error_name,
)
from .publish_result import PUBLISHABLE

R = Calibrate.Result
FB = Calibrate.Feedback
MODES = (Calibrate.Goal.MODE_EYE_TO_HAND, Calibrate.Goal.MODE_EYE_IN_HAND)
EXECUTIONS = (Calibrate.Goal.EXECUTION_MOVEIT, Calibrate.Goal.EXECUTION_MANUAL)
PHASES = {FB.PHASE_SEED_CHECK, FB.PHASE_BOOTSTRAP, FB.PHASE_CENTRING, FB.PHASE_PLANNING,
          FB.PHASE_MOVING, FB.PHASE_CAPTURING, FB.PHASE_SOLVING, FB.PHASE_VERIFYING}
# session failure reasons (02-features §B) -> result codes (01-architecture §5)
SESSION_CODES = {S.MOTION_FAILED_REASON: R.MOTION_FAILED, S.RECOVERY_FAILED: R.MOTION_FAILED,
                 S.SEED_REJECTED: R.NOT_ENOUGH_SAMPLES, S.BOARD_LOST: R.NOT_ENOUGH_SAMPLES,
                 S.BOOTSTRAP_FAILED: R.NOT_ENOUGH_SAMPLES,
                 S.NOT_ENOUGH_SAMPLES: R.NOT_ENOUGH_SAMPLES}


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
    motion_note: str = ''               # why the last motion was not 'ok' (for the message)
    action_cancel: bool = False         # the cancel came through the action (not ~/cancel)
    execution: str = ''                 # the goal's execution (manual / moveit)


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
        # the jump callback missed small sim rewinds on Lyrical in CI: under sim time
        # also watch /clock itself and count any backwards step
        self._last_clock = -1.0
        if self.get_parameter('use_sim_time').value:
            self.create_subscription(Clock, '/clock', self._on_clock,
                                     QoSProfile(depth=10,
                                                reliability=ReliabilityPolicy.BEST_EFFORT),
                                     callback_group=self._sensors)
        self.last_run_records: list[CaptureRecord] = []
        self.last_result: Calibrate.Result | None = None
        self._solve_end: Any = None          # node time the last final solve finished
        self._joints: list[tuple[float, JointState]] = []
        self._still_watches: list[StillWatch] = []   # joints watched during a goal
        self._joint_names_seen: set[str] = set()   # every joint ever reported
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10,
                                 callback_group=self._sensors)
        # the URDF, for the carried-object geometry warning (§3.4)
        self._robot_description = ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/robot_description',
                                 lambda m: setattr(self, '_robot_description', m.data), latched,
                                 callback_group=self._sensors)
        # MoveIt services and actions (§3): their replies arrive on this group
        # while the run thread waits for them
        self.moveit = MoveItClient(self, ReentrantCallbackGroup())

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
        # stops the active run from anywhere: an action client whose goal reply
        # was lost (DDS discovery under load) has no goal handle to cancel with
        self.create_service(Trigger, '~/cancel', self._srv_cancel,
                            callback_group=MutuallyExclusiveCallbackGroup())
        if self.get_parameter('testing.allow_fault_injection').value:
            self.get_logger().warning('testing.allow_fault_injection is ON: ~/inject_fault '
                                      'can latch a motion fault (tests only)')
            self.create_service(Trigger, '~/inject_fault', self._srv_inject_fault,
                                callback_group=self._server)
        self.get_logger().info('ready: send a goal to ~/calibrate')

    # -- small helpers ------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_clock(self, msg: Clock) -> None:
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        back = t < self._last_clock
        self._last_clock = t
        if back:
            self._on_jump(None)

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
        now = time.monotonic()
        with self._lock:
            self._joints.append((now, msg))
            # the last 5 s (fault clearing needs 0.5 s of samples even from a
            # 500 Hz driver), bounded in size
            if len(self._joints) > 64 and self._joints[0][0] < now - 5.0:
                self._joints = [(t, m) for t, m in self._joints if t >= now - 5.0]
            self._joints = self._joints[-10000:]
            self._joint_names_seen.update(msg.name)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for w in self._still_watches:
                w.sample(now, stamp, msg)

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
                                     profile_text=profile_text, execution=goal.execution)
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
                self._run.action_cancel = True
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
            if run is None or run.execution != Calibrate.Goal.EXECUTION_MANUAL or \
                    run.phase not in (FB.PHASE_MANUAL_CAPTURE, FB.PHASE_CAPTURING):
                res.success, res.message = False, 'not in the manual capture phase'
            elif run.accepted < run.min_captures:
                res.success, res.message = False, (
                    f'{run.accepted} accepted captures; at least {run.min_captures} needed')
            else:
                run.finish_request.set()
                res.success, res.message = True, 'solving after the current capture'
        return res

    def _srv_cancel(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        """Cancel the active run, as an action cancel would (refused once the run
        is finalizing); its goal ends CANCELED (or MOTION_FAULT if a motion's stop
        cannot be confirmed)."""
        with self._lock:
            run = self._run
            if run is None:
                res.success, res.message = False, 'no active run'
            elif run.finalizing:
                res.success, res.message = False, 'the run is finalizing; its outcome is decided'
            else:
                run.cancelled.set()
                res.success, res.message = True, 'cancelling the active run'
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
        if self.moveit.active():
            # §6: a motion goal of ours that never reached a terminal state
            res.success, res.message = False, ('a motion goal from this node is still active '
                                               'in MoveIt; stop it (or restart move_group)')
            return res
        t0, call_now = time.monotonic(), self.now()
        window, vmax, deadline, min_msgs = 0.5, 0.01, t0 + 3.0, 5
        waiting = ''
        while time.monotonic() < deadline:
            with self._lock:
                # fresh: arrived after the call AND stamped after it (not a replay)
                fresh = [(t, m) for t, m in self._joints if t > t0 and
                         m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= call_now]
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
                # every joint the robot has ever reported (also before the call,
                # even if its stream stopped) needs its own fresh evidence: 5
                # samples spanning the window
                with self._lock:
                    known = set(self._joint_names_seen)
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
                if self.moveit.active():         # a late goal appeared meanwhile
                    res.success, res.message = False, 'a motion goal from this node is active'
                    return res
                # goals that ended without a terminal status are covered by this
                # stillness evidence
                self.moveit.forget_unconfirmed()
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
            result = self._final_verdict(run, self._result(
                R.INTERNAL, f'internal error: {e!r}', counts=(run.accepted, run.rejected)))
            self._terminate(goal_handle, result)     # an accepted cancel is honoured here too
            return result
        finally:
            with self._lock:
                self._run = None             # first: nothing below may keep the slot busy
            self.last_run_records = list(run.records)
            self.last_result = locals().get('result')       # for diagnostics and tests
            r = self.last_result
            # (rclpy fixes one severity per log call site: two separate calls)
            if r is not None and r.error_code == R.NONE:
                self.get_logger().info(f'run ended: {r.message}')
            elif r is not None:
                self.get_logger().warning(f'run ended: code {r.error_code}: {r.message}')

    def _terminate(self, goal_handle: Any, result: Calibrate.Result) -> None:
        """The one place a goal reaches its terminal state. Only a success leaves
        the result publishable: the marker is created after the final verdict."""
        # publishable only when the run SUCCEEDED and every quality gate passed;
        # a calibration that failed its gates is kept for inspection, not published
        if result.error_code == R.NONE and result.passed and result.result_file:
            try:
                (Path(result.result_file).parent / PUBLISHABLE).touch()
            except OSError as e:
                self.get_logger().error(f'result kept but not publishable: {e}')
        run = self._run
        if result.error_code == R.CANCELED and (run is None or run.action_cancel):
            # the cancel callback runs before rclpy moves the goal to CANCELING:
            # wait for that transition so canceled() is legal (a cancel through
            # ~/cancel has no such transition: that goal ends ABORTED / CANCELED)
            t0 = time.monotonic()
            while not goal_handle.is_cancel_requested and time.monotonic() - t0 < 2.0:
                time.sleep(0.01)
        # pass the result: rclpy's succeed()/abort()/canceled() without one send a
        # DEFAULT (empty) result to any client already waiting for it
        if result.error_code == R.CANCELED and goal_handle.is_cancel_requested:
            goal_handle.canceled(result)
        elif result.error_code == R.NONE:
            goal_handle.succeed(result)
        else:
            goal_handle.abort(result)

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
        new = None
        if cancel and result.error_code not in (R.CANCELED, R.MOTION_FAULT):
            # (a cancel whose stop was not confirmed stays MOTION_FAULT, §6)
            new = self._result(R.CANCELED, 'cancelled' + kept, result_file=result.result_file)
        elif result.error_code == R.NONE and jumped:
            new = self._result(R.TF_UNAVAILABLE, 'the node clock jumped back during the '
                               'run; the result is not trusted' + kept,
                               result_file=result.result_file)
        elif result.error_code == R.NONE and info_changed:
            new = self._result(R.NO_CAMERA_INFO, 'CameraInfo changed during the run; the '
                               'result is not trusted' + kept, result_file=result.result_file)
        if new is None:
            return result
        new.quality = result.quality          # the counts and metrics measured stay reported
        return new

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
                frames: dict[str, str] | None = None, result_file: str = '',
                counts: tuple[int, int] | None = None) -> Calibrate.Result:
        r = R()
        r.error_code, r.message, r.result_file = code, message, result_file
        r.quality = quality_msg(cal)
        if cal is None and counts is not None:       # captures taken before the run ended
            r.quality.captures_accepted, r.quality.captures_rejected = counts
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

    def shutdown_timeout(self) -> float:
        """How long a shutdown waits for a run to end: a motion's cancel
        confirmation plus margin."""
        try:
            return float(self.get_parameter('moveit.cancel_timeout').value) + 5.0
        except Exception:  # noqa: BLE001
            return 10.0

    def wait_idle(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while self._run is not None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self._run is not None:
            self.get_logger().error('shutting down with a run still active: check the robot')
        return self._run is None

    def _run_goal(self, goal_handle: Any, goal: Calibrate.Goal, run: RunState
                  ) -> Calibrate.Result:
        self._solve_end = None
        try:
            cfg = P.load_run_config(self, goal.profile, run.snapshot, run.profile_text)
        except ValueError as e:
            return self._result(R.INVALID_GOAL, str(e))
        try:
            cfg = self._resolve_box(cfg, run)
        except RunAbort as e:
            return self._result(e.code, str(e))
        try:                # the box this run uses (a disabled box is removed)
            centre = tf.make(Rotation.from_euler('xyz', cfg.session.box_rpy),
                             np.asarray(cfg.session.box_xyz, float))
            self.pub_markers.publish(viz.box_marker(
                centre, cfg.session.box_size, cfg.frames['base_frame'],
                self.get_clock().now().to_msg(), cfg.session.box_enabled))
        except Exception as e:  # noqa: BLE001 - visualisation must never end a run
            self.get_logger().warning(f'safety box marker failed: {e!r}')
        with self._lock:
            # ~/finish needs enough captures for every k-fold training split
            run.min_captures = P.least_captures(cfg.solver.min_captures, cfg.solver.folds)
            run.target_captures = cfg.session.target_captures
        extra = ''
        try:
            if goal.execution == Calibrate.Goal.EXECUTION_MOVEIT and cfg.moveit.missing():
                raise RunAbort(R.INVALID_GOAL, f'MoveIt execution needs {cfg.moveit.missing()}')
            camera, info = self._preflight(goal_handle, run, cfg)
            frames = dict(cfg.frames, camera_frame=info.header.frame_id)
            t_solve = self.now()
            if goal.execution == Calibrate.Goal.EXECUTION_MOVEIT:
                sres = self._moveit(goal_handle, run, cfg, camera, info, goal.mode)
                records = sres.records
                self._solve_end = self.get_clock().now()
                self._check_cancel(goal_handle, run)
                self._check_jump(t_solve)
                extra = sres.message
                if sres.calibration is None:
                    code = SESSION_CODES.get(sres.failure or '', R.INTERNAL)
                    note = f' ({run.motion_note})' if code == R.MOTION_FAILED and \
                        run.motion_note else ''
                    return self._result(code, f'{sres.failure}: {sres.message}{note}',
                                        counts=(run.accepted, run.rejected))
                cal = sres.calibration
            else:
                records = self._manual(goal_handle, run, cfg, camera, info)
                self._feedback(goal_handle, run, FB.PHASE_SOLVING)
                t_solve = self.now()
                cal = calibrate(goal.mode, records, cfg.solver,
                                on_solved=lambda: self._feedback(goal_handle, run,
                                                                 FB.PHASE_VERIFYING))
                self._solve_end = self.get_clock().now()
                # a cancel or a clock reset during the solve still wins (no result written)
                self._check_cancel(goal_handle, run)
                self._check_jump(t_solve)
        except RunAbort as e:
            if e.code == R.MOTION_FAULT:
                with self._lock:
                    self._fault_latched = True
                self.get_logger().error(f'MOTION FAULT latched: {e}. New goals are refused '
                                        'until ~/clear_fault succeeds')
            return self._result(e.code, str(e), counts=(run.accepted, run.rejected))
        path = self._write_results(goal.mode, cfg, records, cal, frames)
        if extra:
            extra = f' ({extra.strip()})'
        if cal.solution is None:
            reason = cal.failure.reason if cal.failure else 'unknown'
            msg = f'solve failed ({reason}): {cal.failure}; observations saved for an ' \
                  'offline re-solve'
            # 02-features §E5: too_few_captures -> NOT_ENOUGH_SAMPLES, others SOLVE_FAILED
            code = R.NOT_ENOUGH_SAMPLES if reason == TOO_FEW_CAPTURES else R.SOLVE_FAILED
            return self._result(code, msg + extra, cal, frames, str(path))
        msg = 'calibration passed all quality gates' if cal.passed else \
            'calibration finished but failed quality gates: ' + ', '.join(
                g.name for g in cal.report.gates if g.gated and not g.passed)
        return self._result(R.NONE, msg + extra, cal, frames, str(path))

    # -- preflight (§7-8) -------------------------------------------------------------
    def _preflight(self, goal_handle: Any, run: RunState, cfg: P.RunConfig
                   ) -> tuple[CameraModel, CameraInfo]:
        self._feedback(goal_handle, run, FB.PHASE_PREFLIGHT)
        t = cfg.timing
        start = self.now()
        start_count = self.images.count      # arrivals counted: works on a paused clock too
        deadline = time.monotonic() + t.camera_info_timeout
        while True:
            self._check_cancel(goal_handle, run)
            with self._info_lock:
                info = self._info
            img = self.images.latest()
            # an image that ARRIVED after this goal started: a stopped camera's
            # cached last frame must not pass the preflight
            if info is not None and img is not None and self.images.count > start_count:
                break
            self._check_jump(start)          # a reset here is reported as such
            if time.monotonic() > deadline:
                what = 'CameraInfo' if info is None else 'fresh images'
                raise RunAbort(R.NO_CAMERA_INFO, f'no {what} received within '
                               f'{t.camera_info_timeout:.0f} s; check the camera topics '
                               '(remap "image" and "camera_info")')
            time.sleep(0.05)
        base, tool = cfg.frames['base_frame'], cfg.frames['tool_frame']
        # TF at an image's stamp, retried with the NEWEST image each time (an early
        # frame can predate this node's TF history), until the deadline
        tf_deadline = time.monotonic() + max(t.tf_timeout, t.camera_info_timeout)
        found = None
        clock_error: RunAbort | None = None
        while found is None and time.monotonic() < tf_deadline:
            self._check_jump(start)          # a reset here is reported as such
            self._drop_future_tf(base, tool, cfg)
            self._check_cancel(goal_handle, run)
            img = self.images.latest() or img
            with self._info_lock:            # the newest CameraInfo too: around a camera
                info = self._info or info    # change, image and info arrive in either order
            try:
                # each newest frame must match the pinned CameraInfo and the clock; a
                # queued frame from just before a camera change or a sim reset is
                # skipped, and only the deadline turns a mismatch into an abort
                if img.frame_id != info.header.frame_id:
                    raise RunAbort(R.NO_CAMERA_INFO, f'image frame {img.frame_id!r} differs '
                                   f'from CameraInfo frame {info.header.frame_id!r}')
                if img.gray.shape[1] != info.width or img.gray.shape[0] != info.height:
                    raise RunAbort(R.NO_CAMERA_INFO, 'image size differs from CameraInfo')
                self._check_clock(img.stamp, img.received, cfg)
            except RunAbort as e:
                clock_error = e
                time.sleep(0.05)
                continue
            clock_error = None
            found = self._lookup_cancellable(goal_handle, run, base, tool, img.stamp_ns,
                                             t.tf_timeout)      # full latency budget per frame
            with self._info_lock:
                newer = self._info
            if found is not None and newer is not None and not same_info(newer, info):
                found = None                 # CameraInfo changed during the TF wait: retry
        self._check_jump(start)              # a reset during the last lookup, too
        if found is None and clock_error is not None:
            raise clock_error
        if found is None:
            if tf_tools.latest(self.tf_buffer, base, tool) is None:
                why = 'is the robot driver (robot_state_publisher) running?'
            else:
                why = ('TF exists but not at the image stamp: the robot and the camera may '
                       'use different clocks (use_sim_time / device clocks)')
            raise RunAbort(R.TF_UNAVAILABLE, f'no TF {base} -> {tool} at the image stamp; '
                           + why)
        # pin the CameraInfo of the matching (image, info) pair just used
        try:
            camera = camera_model(info, cfg.image_is_rectified)
        except CameraInfoError as e:
            raise RunAbort(R.NO_CAMERA_INFO, str(e)) from e
        with self._lock:
            run.info = info                  # pinned now: any later change taints the run
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

    # -- MoveIt mode (§3) ---------------------------------------------------------------
    def _moveit(self, goal_handle: Any, run: RunState, cfg: P.RunConfig, camera: CameraModel,
                info: CameraInfo, mode: str) -> S.SessionResult:
        """Drive the core session: it asks for moves and captures, we execute them
        through MoveIt and report measured outcomes (02-features §B)."""
        mi = cfg.moveit
        base, tool = cfg.frames['base_frame'], cfg.frames['tool_frame']
        ik_link = mi.ik_link or tool
        missing = self.moveit.missing(mi.service_timeout, run.cancelled)
        self._check_cancel(goal_handle, run)
        if 'compute_ik' in missing:
            raise RunAbort(R.IK_SERVICE_MISSING, 'no compute_ik service (is move_group '
                           'running? remap "compute_ik" if it has another name); use '
                           'execution "manual" without MoveIt')
        if missing:
            raise RunAbort(R.MOTION_FAILED, f'MoveIt interfaces missing: {", ".join(missing)}'
                           ' (is move_group running with its default capabilities?)')
        ik_T_tool = self._fixed_offset(goal_handle, run, ik_link, tool, cfg)
        tool_T_ik = tf.inv(ik_T_tool)
        self._warn_carried_geometry(cfg, ik_link)
        if self._joints_now(mi.joint_names, time.monotonic(), 3.0, run) is None:
            raise RunAbort(R.TF_UNAVAILABLE, 'no /joint_states for '
                           f'{", ".join(mi.joint_names)}: is the robot driver running?')
        path = None
        if cfg.session.box_enabled and mi.box_path_constraint:
            centre = tf.make(Rotation.from_euler('xyz', cfg.session.box_rpy),
                             np.asarray(cfg.session.box_xyz, float))
            path = self.moveit.box_constraint(ik_link, ik_T_tool, base, centre,
                                              cfg.session.box_size)

        def ik(G: Any) -> IKResult:
            self._check_cancel(goal_handle, run)
            return self.moveit.solve_ik(G @ tool_T_ik, base, mi.group_name, ik_link,
                                        mi.joint_names, mi.ik_timeout, run.cancelled)

        def on_phase(phase: str) -> None:
            if phase in PHASES:
                self._feedback(goal_handle, run, phase)

        session = S.Session(mode, cfg.board, camera, ik, cfg=cfg.session,
                            capture_cfg=cfg.capture, solver_cfg=cfg.solver, on_phase=on_phase)
        last_now = self.now()
        weak_reported = False
        while True:
            self._check_cancel(goal_handle, run)
            self._check_jump(last_now)
            last_now = max(last_now, self.now())
            self._check_info(run)
            action = session.next_action()
            if isinstance(action, S.Finished):
                return action.result
            if session.plan is not None and session.plan.weak and not weak_reported:
                weak_reported = True        # §C6: reported before execution, still executed
                self.get_logger().warning(f'weak plan: {session.plan.advice}')
                with self._lock:
                    run.last_detail = f'weak plan: {session.plan.advice}'[:300]
                self._feedback(goal_handle, run)
            if isinstance(action, S.Move):
                outcome, joints, pose = self._motion(goal_handle, run, cfg, action, ik_link,
                                                     tool_T_ik, path, mode,
                                                     session.check_pose)
                session.on_motion(outcome, joints, pose)
                continue
            frames, images, _, detail = self._collect(goal_handle, run, cfg, info)
            joints = self._joints_now(mi.joint_names, time.monotonic(), 1.0, run)
            n_before = len(session.records)
            session.on_capture(frames, joints)        # may run the final solve
            recs = session.records
            rec = recs[n_before] if len(recs) > n_before else None
            if rec is None:
                continue
            self._check_info(run)
            run.records = recs
            with self._lock:
                if rec.accepted:
                    run.accepted += 1
                    run.last_rejection, run.last_detail = '', ''
                else:
                    run.rejected += 1
                    run.last_rejection = rec.reject or ''
                    run.last_detail = rec.detail or detail
            self._show(images, rec, cfg, camera)
            self._feedback(goal_handle, run)

    def _resolve_box(self, cfg: P.RunConfig, run: RunState) -> P.RunConfig:
        """§3.4: a box given in a named frame becomes a base-frame box (looked up
        once at goal start; the frame must be in the same TF tree)."""
        sc = cfg.session
        base = cfg.frames['base_frame']
        if not sc.box_enabled or not sc.box_frame or sc.box_frame == base:
            return cfg
        got = None
        deadline = time.monotonic() + 2.0
        while got is None and time.monotonic() < deadline and not run.cancelled.is_set():
            got = tf_tools.latest(self.tf_buffer, base, sc.box_frame)
            if got is None:
                time.sleep(0.05)
        if got is None:
            raise RunAbort(R.TF_UNAVAILABLE, f'no TF {base} -> {sc.box_frame} '
                           '(session.box_frame) for the safety box')
        B = got[0] @ tf.make(Rotation.from_euler('xyz', sc.box_rpy),
                             np.asarray(sc.box_xyz, float))
        box = replace(sc, box_frame='', box_xyz=tuple(float(v) for v in B[:3, 3]),
                      box_rpy=tuple(float(v) for v in
                                    Rotation.from_matrix(B[:3, :3]).as_euler('xyz')))
        return replace(cfg, session=box)

    def _warn_carried_geometry(self, cfg: P.RunConfig, ik_link: str) -> None:
        """§3.4: warn when the carried board or camera has no collision geometry:
        neither an attached collision object on a link rigid with the tool, nor
        a URDF link named in ``moveit.carried_links`` that has collision
        geometry (the robot's own hand geometry is not the carried object, so
        URDF geometry counts only when named). MoveIt then can't keep it out of
        collisions."""
        tool = cfg.frames['tool_frame']
        urdf = self._robot_description
        rigid = _fixed_descendants(urdf, {tool, ik_link}) | {tool, ik_link}
        attached = self.moveit.attached_links(cfg.moveit.service_timeout)
        if attached is not None and attached & rigid:
            return
        named = [link for link in cfg.moveit.carried_links if _has_collision(urdf, link)]
        if named:
            return
        self.get_logger().warning(
            'no collision geometry for the carried board or camera: no attached collision '
            f'object on {sorted(rigid)} and no moveit.carried_links with URDF collision '
            'geometry. MoveIt cannot keep it out of collisions; attach it to the planning '
            'scene (or name its URDF link in moveit.carried_links).')

    def _fixed_offset(self, goal_handle: Any, run: RunState, ik_link: str, tool: str,
                      cfg: P.RunConfig) -> Any:
        """§3.1: the ik_link -> tool_frame transform, which must be fixed: looked up
        over 1 s (5 samples, so a periodic motion cannot alias into two equal
        lookups), all identical."""
        if ik_link == tool:
            return np.eye(4)
        seen = []
        for k in range(5):
            got = None
            deadline = time.monotonic() + max(2.0, cfg.timing.tf_timeout)
            while got is None and time.monotonic() < deadline:
                self._check_cancel(goal_handle, run)
                got = tf_tools.latest(self.tf_buffer, ik_link, tool)
                if got is None:
                    time.sleep(0.05)
            if got is None:
                raise RunAbort(R.TF_UNAVAILABLE, f'no TF {ik_link} -> {tool}: the tool frame '
                               'must be rigidly attached to moveit.ik_link')
            seen.append(got[0])
            if k < 4:
                t0 = time.monotonic()
                while time.monotonic() - t0 < 0.25:
                    self._check_cancel(goal_handle, run)
                    time.sleep(0.05)
        if any(tf.translation_distance(seen[0], T) > 1e-6 or
               tf.rotation_angle(seen[0], T) > 1e-6 for T in seen[1:]):
            raise RunAbort(R.TF_UNAVAILABLE, f'TF {ik_link} -> {tool} is moving; the tool '
                           'frame must be fixed to moveit.ik_link')
        return seen[-1]

    def _joints_now(self, names: Any, after: float, timeout: float, run: RunState
                    ) -> Any:
        """Measured joints (in ``names`` order) from /joint_states received after
        ``after`` (monotonic) AND stamped after the call (a replayed or queued old
        message is not evidence), newest per joint; None if not all arrive in
        time or a value is not finite."""
        deadline = time.monotonic() + timeout
        call_now = self.now()
        while True:
            with self._lock:
                msgs = [m for t, m in self._joints if t > after and
                        m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= call_now]
            pos: dict[str, float] = {}
            for m in msgs:                      # oldest first: newer values win
                pos.update(zip(m.name, m.position, strict=False))
            if all(n in pos for n in names):
                q = np.array([pos[n] for n in names], dtype=float)
                if np.all(np.isfinite(q)):
                    return q
            if time.monotonic() > deadline or run.cancelled.is_set():
                return None
            time.sleep(0.02)

    def _motion(self, *args: Any, **kwargs: Any) -> tuple[str, Any, Any]:
        """_motion_body, and no joint watch outlives it (exceptions included)."""
        try:
            return self._motion_body(*args, **kwargs)
        finally:
            with self._lock:
                self._still_watches.clear()

    def _motion_body(self, goal_handle: Any, run: RunState, cfg: P.RunConfig, a: S.Move,
                     ik_link: str, tool_T_ik: Any, path: Any, mode: str,
                     check_pose: Any = None) -> tuple[str, Any, Any]:
        """One Move (§3.2) -> (outcome, measured joints, measured tool pose).

        ok: goal accepted, SUCCEEDED with SUCCESS, and the measured TF tool pose
        (and, for joint goals, the joints) at the target after settling: a
        planning-only move_group or a clamped joint goal reports success without
        getting there. not_started: provably nothing moved (a refused Cartesian
        path; a refused plan; an ABORTED MoveGroup goal with an empty planned
        trajectory and unchanged joints). Anything else is failed. A cancel ends
        the run (CANCELED), or MOTION_FAULT when the stop is not confirmed (§6).

        With the box also guarding the carried object, a planned move is planned
        only, its whole path is checked (the L_max ball inside the box, through
        FK), and only then executed (01-architecture §3.2)."""
        mi = cfg.moveit
        base, tool = cfg.frames['base_frame'], cfg.frames['tool_frame']
        with self._lock:
            run.motion_note = ''
        t_start = self.now()
        before = self._joints_now(mi.joint_names, time.monotonic(), 2.0, run)
        check_path = cfg.session.box_enabled and cfg.session.box_check_moving_object
        watch: StillWatch | None = None

        def invalid() -> bool:                # the run stopped being valid (§6-8)
            return run.info_changed or self._jumps != run.jumps
        if a.cartesian:
            plan = self.moveit.plan_cartesian(
                [W @ tool_T_ik for W in a.waypoints], base, mi.group_name, ik_link,
                mi.cartesian_max_step, mi.revolute_jump_threshold, mi.velocity_scaling,
                mi.acceleration_scaling, mi.service_timeout + 10.0, run.cancelled)
            self._check_cancel(goal_handle, run)
            # the path MoveIt returns is re-timed and resampled, so a jump check on its
            # points is not enough: the measured tool path must be the straight one
            # the core checked (01-architecture §3.2)
            why = plan.note if not plan.ok else self._off_straight_path(
                plan.trajectory, a.waypoints, a.tool_pose, cfg, ik_link, tool_T_ik, run,
            check_pose) or \
                self.moveit.invalid_state(plan.trajectory, mi.group_name, mi.service_timeout,
                                          run.cancelled)
            self._check_cancel(goal_handle, run)
            if why:                               # nothing was sent (probe finding 7)
                return self._not_started(run, cfg, why, before)
            self._still_valid(goal_handle, run, t_start)     # right before anything moves
            out = self.moveit.run(self.moveit.execute,
                                  self.moveit.execute_goal(plan.trajectory),
                                  mi.motion_timeout, mi.cancel_timeout, run.cancelled, invalid)
        else:
            goal = self.moveit.joint_goal(a.joints, mi.joint_names, mi.group_name, base,
                                          mi.planning_time, mi.velocity_scaling,
                                          mi.acceleration_scaling, path)
            if check_path:
                goal.planning_options.plan_only = True
                self._still_valid(goal_handle, run, t_start)
                out = self.moveit.run(self.moveit.move, goal, mi.planning_time + 10.0,
                                      mi.cancel_timeout, run.cancelled, invalid)
                self._stopped(out, run)
                if out.stopped == 'invalid':
                    self._still_valid(goal_handle, run, t_start)
                after = self._joints_now(mi.joint_names, time.monotonic(), 2.0, run)
                moved = not _same(before, after, mi.joint_tolerance)
                if not out.accepted or moved:     # a plan-only goal must not move anything
                    return self._failed(run, out.note or 'the robot moved during a plan-only '
                                        'request', after)
                if out.status != GoalStatus.STATUS_SUCCEEDED or out.error_code != SUCCESS or \
                        out.planned is None or not out.planned.joint_trajectory.points:
                    return self._not_started(run, cfg, 'planning failed '
                                             f'({error_name(out.error_code)})', after)
                why = self._path_outside_box(out.planned, cfg, ik_link, tool_T_ik, mode, run) \
                    or self.moveit.invalid_state(out.planned, mi.group_name, mi.service_timeout,
                                                 run.cancelled)
                self._check_cancel(goal_handle, run)
                if why:
                    return self._not_started(run, cfg, why, after)
                self._still_valid(goal_handle, run, t_start)
                out = self.moveit.run(self.moveit.execute,
                                      self.moveit.execute_goal(out.planned),
                                      mi.motion_timeout, mi.cancel_timeout, run.cancelled,
                                      invalid)
            else:
                self._still_valid(goal_handle, run, t_start)
                if before is not None:   # evidence for a safe skip (§3.2), kept
                    # until the reply's fresh joints are in (_not_started)
                    watch = StillWatch(dict(zip(mi.joint_names, map(float, before),
                                                strict=True)),
                                       mi.joint_tolerance, time.monotonic(), self.now())
                    with self._lock:
                        self._still_watches.append(watch)
                out = self.moveit.run(self.moveit.move, goal,
                                      mi.planning_time + mi.motion_timeout,
                                      mi.cancel_timeout, run.cancelled, invalid)
        self._stopped(out, run)                   # raises on a cancel or an unconfirmed stop
        if out.stopped == 'invalid':              # stopped because the run became invalid
            self._still_valid(goal_handle, run, t_start)      # raises with the reason
        after = self._joints_now(mi.joint_names, time.monotonic(), 2.0, run)
        if not out.accepted:
            return self._failed(run, out.note, after)
        # INVALID_MOTION_PLAN comes with the rejected plan filled in (MoveIt's
        # ValidateSolution adapter; seen in the mount sweep): a safe skip only if
        # the joints were watched from the goal until the reply's fresh joints and
        # never left the start (§3.2, checked in _not_started). shortcut: sampled
        # at the driver's rate, a gap of at most STILL_MAX_GAP s is trusted;
        # tighten if a robot can leave and return within that.
        rejected = out.error_code == INVALID_MOTION_PLAN and watch is not None
        if not a.cartesian and not check_path and out.status == GoalStatus.STATUS_ABORTED and \
                (out.planned_points == 0 or rejected) and \
                _same(before, after, mi.joint_tolerance):
            return self._not_started(run, cfg, f'planning failed ({error_name(out.error_code)})',
                                     after, watch if out.planned_points != 0 else None)
        if out.status != GoalStatus.STATUS_SUCCEEDED or out.error_code != SUCCESS:
            return self._failed(run, f'MoveIt ended the goal with status {out.status}, '
                                f'{error_name(out.error_code)}', after)
        # end-state check: where the robot really is, measured after settling
        t = cfg.timing
        s = tf_tools.settle(self.tf_buffer, base, tool, self.now, t.settle_window,
                            t.settle_translation, t.settle_rotation, t.settle_timeout,
                            run.cancelled)
        self._check_cancel(goal_handle, run)
        if not s.ok or s.pose is None:
            return self._failed(run, f'no settled measurement after the motion: {s.reason}',
                                after)
        after = self._joints_now(mi.joint_names, time.monotonic(), 2.0, run)
        if after is None:            # 'ok' always carries fresh measured joints (§3.2)
            return self._failed(run, 'no fresh /joint_states after the motion', None, s.pose)
        dt = tf.translation_distance(s.pose, a.tool_pose)
        dr = tf.rotation_angle(s.pose, a.tool_pose)
        if not (dt <= mi.pose_tolerance_translation and dr <= mi.pose_tolerance_rotation):
            return self._failed(run, f'MoveIt reported success but the measured tool pose is '
                                f'{dt * 1000:.1f} mm / {math.degrees(dr):.2f} deg from the '
                                'target', after, s.pose)
        if not a.cartesian and not _same(after, np.asarray(a.joints), mi.joint_tolerance):
            return self._failed(run, 'MoveIt reported success but the measured joints are not '
                                'at the goal', after, s.pose)
        return S.MOTION_OK, after, s.pose

    def _still_valid(self, goal_handle: Any, run: RunState, since: float) -> None:
        """Right before a goal that can move the robot: the run is still valid
        (not cancelled, no clock reset, CameraInfo unchanged)."""
        self._check_cancel(goal_handle, run)
        self._check_jump(since)
        self._check_info(run)

    def _off_straight_path(self, traj: Any, waypoints: list[Any], target: Any,
                           cfg: P.RunConfig, ik_link: str, tool_T_ik: Any,
                           run: RunState, check_pose: Any) -> str:
        """A checked short move must follow the straight tool path the core
        checked: through FK of the planned trajectory (joints interpolated at
        <= 0.01 rad), every tool position within ``pose_tolerance_translation``
        of the waypoint polyline and every orientation within
        ``pose_tolerance_rotation`` of the nearest waypoint's, beyond the half
        waypoint spacing. A joint-space branch flip swings the tool off it. ''
        if on the path, else why."""
        mi = cfg.moveit
        jt = traj.joint_trajectory
        q = np.array([list(p.positions) for p in jt.points], dtype=float)
        if len(q) == 0 or not np.all(np.isfinite(q)):
            return 'the Cartesian path has no or invalid points'
        samples = dense_samples(q)
        poses = self.moveit.fk_poses(list(jt.joint_names), samples, ik_link,
                                     cfg.frames['base_frame'], mi.service_timeout,
                                     run.cancelled)
        if poses is None:
            return 'compute_fk failed while checking the Cartesian path'
        path = [np.asarray(W, float) for W in waypoints] or [np.asarray(target, float)]
        P0 = poses[0] @ tf.inv(tool_T_ik)
        path = [P0] + path                  # from where the robot is
        half = max((tf.rotation_angle(u, v) for u, v in zip(path[:-1], path[1:],
                                                             strict=True)), default=0.0) / 2
        ik_T_tool = tf.inv(tool_T_ik)
        for P_ik in poses:
            G = P_ik @ ik_T_tool
            d = min(_segment_distance(G[:3, 3], u[:3, 3], v[:3, 3])
                    for u, v in zip(path[:-1], path[1:], strict=True)) if len(path) > 1 else \
                float(np.linalg.norm(G[:3, 3] - path[0][:3, 3]))
            r = min(tf.rotation_angle(G, W) for W in path)
            if d > mi.pose_tolerance_translation or r > half + mi.pose_tolerance_rotation:
                return (f'the Cartesian path leaves the straight tool path by '
                        f'{d * 1000:.1f} mm / {math.degrees(max(0.0, r - half)):.2f} deg '
                        '(a joint-space jump)')
            # the core checked the nominal waypoints; the path really driven may
            # deviate within the tolerance, so every limit (box, carried object,
            # seed envelope, orientation) is checked on it too, exactly (FK poses)
            why = check_pose(G)
            if why:
                return f'the Cartesian path as planned breaks a limit ({why})'
        return ''

    def _path_outside_box(self, traj: Any, cfg: P.RunConfig, ik_link: str, tool_T_ik: Any,
                          mode: str, run: RunState) -> str:
        """The carried object's L_max ball stays inside the box along the planned
        path: joints interpolated linearly between trajectory points, FK at every
        sample, 1 cm inside the faces (the controller follows splines between the
        points, close to but not exactly the linear path). '' if inside, else why.
        ``shortcut:`` sampled at <= 0.01 rad joint steps, not swept; tighten if a
        real box face is ever touched between samples."""
        jt = traj.joint_trajectory
        q = np.array([list(p.positions) for p in jt.points], dtype=float)
        if not np.all(np.isfinite(q)):
            return 'the planned trajectory has invalid joint values'
        samples = dense_samples(q)
        poses = self.moveit.fk_poses(list(jt.joint_names), samples, ik_link,
                                     cfg.frames['base_frame'], cfg.moveit.service_timeout,
                                     run.cancelled)
        if poses is None:
            return 'compute_fk failed while checking the planned path'
        ik_T_tool = tf.inv(tool_T_ik)
        for P_ik in poses:
            # the controller follows splines between the points, not straight
            # joint lines: a margin covers that (shortcut: fixed 1 cm, not a bound)
            why = _box_violation(P_ik @ ik_T_tool, cfg, mode, PATH_MARGIN)
            if why:
                return f'the planned path {why}'
        return ''

    def _stopped(self, out: GoalOutcome, run: RunState) -> None:
        """A goal we could not see to its end: CANCELED when its stop is confirmed,
        else a fault."""
        if out.sent and not out.answered:
            raise RunAbort(R.MOTION_FAULT, 'MoveIt did not answer a motion goal in time; it '
                           'may still accept and run it (it is cancelled if it does); check '
                           'the robot')
        if not out.stopped:
            if out.accepted and out.status not in TERMINAL:
                raise RunAbort(R.MOTION_FAULT, f'a motion goal ended without a terminal status '
                               f'(status {out.status}); check the robot')
            return
        if not out.stop_confirmed:
            raise RunAbort(R.MOTION_FAULT, f'motion {out.stopped}: the stop was not confirmed '
                           f'({out.note}); check the robot')
        if out.stopped == 'cancel':
            raise RunAbort(R.CANCELED, 'cancelled (motion stopped)')
        if out.stopped == 'invalid':      # the caller raises the run's own reason
            return
        with self._lock:
            run.motion_note = 'the motion did not finish in moveit.motion_timeout; stopped'
        raise RunAbort(R.MOTION_FAILED, run.motion_note)

    def _not_started(self, run: RunState, cfg: P.RunConfig, why: str, joints: Any,
                     watch: StillWatch | None = None) -> tuple[str, Any, Any]:
        """Fresh joints with the reply (the session orders spares from them,
        §3.2); the earlier sample only if none arrive. With ``watch``, the skip
        also needs the joints to have stayed at the start from the goal until
        these fresh joints."""
        fresh = self._joints_now(cfg.moveit.joint_names, time.monotonic(), 2.0, run)
        if fresh is None:            # every reply carries fresh joints (§3.2)
            return self._failed(run, f'{why}; and no fresh /joint_states after it', None)
        if watch is not None:
            with self._lock:
                if watch in self._still_watches:
                    self._still_watches.remove(watch)
                watch.end(time.monotonic())
            if not watch.still() or not _same(fresh, watch.ref_vector(cfg.moveit.joint_names),
                                              cfg.moveit.joint_tolerance):
                return self._failed(run, f'{why}, but the joints were not seen to stay at the '
                                    f'start while the goal was active ({watch.why()})', fresh)
        self.get_logger().info(f'motion not started: {why}')
        joints = fresh
        got = tf_tools.latest(self.tf_buffer, cfg.frames['base_frame'], cfg.frames['tool_frame'])
        return S.MOTION_NOT_STARTED, joints, None if got is None else got[0]

    def _failed(self, run: RunState, why: str, joints: Any, pose: Any = None
                ) -> tuple[str, Any, Any]:
        self.get_logger().error(f'motion failed: {why}')
        with self._lock:
            run.motion_note = why
        return S.MOTION_FAILED, joints, pose

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
        frames, images, code, detail = self._collect(goal_handle, run, cfg, info)
        if not frames:
            return reject_record(cid, camera, code, detail)
        rec = capture(cid, frames, cfg.board, camera, cfg.capture)
        self._check_info(run)               # intrinsics unchanged while these were taken
        self._show(images, rec, cfg, camera)
        return rec

    def _show(self, images: list[Any], rec: CaptureRecord, cfg: P.RunConfig,
              camera: CameraModel) -> None:
        if images:
            shown = min(images, key=lambda im: abs(im.stamp - (rec.stamp or images[-1].stamp)))
            self._publish_viz(shown, rec, cfg, camera)

    def _collect(self, goal_handle: Any, run: RunState, cfg: P.RunConfig, info: CameraInfo
                 ) -> tuple[list[Frame], list[Any], str, str]:
        """Settle, then the capture's frames, each with TF at its own stamp (§8).
        Returns (frames, images, reject code, detail); no frames = rejected."""
        t = cfg.timing
        base, tool = cfg.frames['base_frame'], cfg.frames['tool_frame']
        self._drop_future_tf(base, tool, cfg)
        t_start = self.now()
        s = tf_tools.settle(self.tf_buffer, base, tool, self.now, t.settle_window,
                            t.settle_translation, t.settle_rotation, t.settle_timeout,
                            run.cancelled)
        self._check_cancel(goal_handle, run)
        self._check_jump(t_start)
        if not s.ok:
            if 'newer than the settle start' in s.reason:
                raise RunAbort(R.TF_UNAVAILABLE, s.reason)
            return [], [], codes.TF_UNAVAILABLE, s.reason
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
            return [], images, codes.NO_BOARD if not images else codes.CONSENSUS, \
                f'{len(images)} of {n} images acquired after the settle within ' \
                f'{t.frame_timeout} s'
        frames: list[Frame] = []
        for im in images:
            pose = self._lookup_cancellable(goal_handle, run, base, tool, im.stamp_ns,
                                            t.tf_timeout)
            frames.append(Frame(im.gray, im.stamp, pose))
            self._check_cancel(goal_handle, run)
        return frames, images, '', ''

    def _drop_future_tf(self, base: str, tool: str, cfg: P.RunConfig) -> None:
        """After a sim reset, TF already in flight can refill the cleared buffer with
        old-epoch stamps far 'in the future'; tf2 then rejects fresh TF as old
        data. Seeing that, clear again (static transforms are kept)."""
        got = tf_tools.latest(self.tf_buffer, base, tool)
        if got is not None and got[1] > self.now() + max(1.0, cfg.timing.clock_tolerance):
            self.get_logger().warning('TF from before a clock reset found: clearing it')
            self.tf_buffer.clear()

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
                                      self.dynamic_links,
                                      {cfg.frames['base_frame'], cfg.frames['tool_frame']})
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


PATH_MARGIN = 0.01        # m inside the box for sampled planned paths


def _box_violation(G: Any, cfg: P.RunConfig, mode: str, margin: float) -> str:
    """'' if tool pose G keeps the tool origin (and, with box_check_moving_object,
    the carried object's L_max ball) inside the enabled box, ``margin`` m inside
    its faces; else why."""
    sc = cfg.session
    if not sc.box_enabled:
        return ''
    if not box_contains(G[:3, 3][None], sc, margin):
        return 'takes the tool out of the safety box'
    if sc.box_check_moving_object and \
            not box_contains(lever_ball_points(mode, G, sc, cfg.board), sc, margin):
        return 'takes the carried object (within L_max) out of the safety box'
    return ''


def _urdf_joints(urdf: str) -> list[tuple[str, str, str]]:
    """(type, parent, child) of every URDF joint; [] if it cannot be parsed."""
    import xml.etree.ElementTree as ET
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError:
        return []
    out = []
    for j in root.iter('joint'):
        p, c = j.find('parent'), j.find('child')
        if p is not None and c is not None:
            out.append((j.get('type', ''), p.get('link', ''), c.get('link', '')))
    return out


def _fixed_descendants(urdf: str, roots: set[str]) -> set[str]:
    """Links below ``roots`` through fixed joints only (rigid with the tool)."""
    joints = _urdf_joints(urdf) if urdf else []
    found: set[str] = set()
    todo = list(roots)
    while todo:
        link = todo.pop()
        for kind, parent, child in joints:
            if parent == link and kind == 'fixed' and child not in found | roots:
                found.add(child)
                todo.append(child)
    return found


def _has_collision(urdf: str, link: str) -> bool:
    import xml.etree.ElementTree as ET
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError:
        return False
    return any(lk.get('name') == link and lk.find('collision') is not None
               for lk in root.iter('link'))


def _segment_distance(p: Any, a: Any, b: Any) -> float:
    ab = b - a
    t = 0.0 if float(ab @ ab) < 1e-18 else float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


STILL_MAX_GAP = 0.25     # s, the largest joint-sample gap a StillWatch accepts


@dataclass
class StillWatch:
    """Did the joints stay at ``ref`` from the goal's start until the watch
    ended? Fed from /joint_states under the node's lock. Messages stamped before
    the start are ignored (a replayed or queued old message is not evidence);
    every watched value in any message, partial or full, must be at ``ref``;
    full messages must carry strictly increasing stamps (a re-sent message
    does not count twice) and arrive at most STILL_MAX_GAP s apart in both
    receive time and stamp, from the start to the end."""
    ref: dict[str, float]
    tol: float
    last: float                 # monotonic time of the last full sample (or the start)
    start_stamp: float          # node time at the start
    max_gap: float = 0.0
    moved: bool = False
    samples: int = 0
    last_stamp: float = -math.inf    # stamp of the last full sample

    def sample(self, now: float, stamp: float, msg: Any) -> None:
        if stamp < self.start_stamp:
            return
        pos = {n: float(v) for n, v in zip(msg.name, msg.position, strict=False)
               if n in self.ref}
        if not pos:
            return
        if any(not math.isfinite(v) or abs(v - self.ref[n]) > self.tol for n, v in pos.items()):
            self.moved = True
        if len(pos) == len(self.ref) and stamp > self.last_stamp:
            prev = self.start_stamp if self.samples == 0 else self.last_stamp
            self.max_gap = max(self.max_gap, now - self.last, stamp - prev)
            self.last, self.last_stamp = now, stamp
            self.samples += 1

    def end(self, now: float) -> None:
        self.max_gap = max(self.max_gap, now - self.last)

    def ref_vector(self, names: Any) -> Any:
        return np.array([self.ref[n] for n in names], dtype=float)

    def still(self) -> bool:
        return self.samples > 0 and not self.moved and self.max_gap <= STILL_MAX_GAP

    def why(self) -> str:
        if self.moved:
            return 'a sample left the start'
        if not self.samples:
            return 'no joint samples'
        return f'a {self.max_gap:.2f} s gap between joint samples'


def _same(a: Any, b: Any, tol: float) -> bool:
    """Both joint vectors known, finite, and within ``tol`` (NaN is never 'same')."""
    if a is None or b is None:
        return False
    d = np.abs(np.asarray(a, float) - np.asarray(b, float))
    return bool(np.all(np.isfinite(d)) and float(np.max(d, initial=0.0)) <= tol)


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
    """Ctrl-C (SIGINT) or SIGTERM first ends an active run, which cancels its
    motion goal and waits for the stop, while the executor still runs; only
    then does the node shut down (rclpy's own handler would shut the context
    down at once, and the cancel could never be sent)."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CalibrationNode()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()
    try:
        while not stop.is_set() and spinner.is_alive():
            stop.wait(0.2)
        node.stop()                 # the run cancels its motion and waits for the stop
        node.wait_idle(node.shutdown_timeout())
    except ExternalShutdownException:
        pass
    finally:
        ex.shutdown(timeout_sec=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
