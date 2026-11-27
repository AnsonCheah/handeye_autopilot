"""MoveIt through its own interfaces (01-architecture §3): ``compute_ik``,
``compute_cartesian_path`` (services) and ``move_action`` / ``execute_trajectory``
(actions), with plain rclpy clients.

Reuse: the ``moveit_msgs`` interfaces that ``move_group`` serves. We write the
request construction and the goal bookkeeping (status, planned trajectory,
cancel, which goals are still active). pymoveit2 is not used: its lifecycle API
differs between distro releases (probe finding 9), and what we need from it is
a few message fields.

ROS 2 concepts: a *service* is one request and one reply; an *action* is a
long-running goal with a terminal status (SUCCEEDED, ABORTED, CANCELED) that
can be cancelled
(https://docs.ros.org/en/jazzy/Concepts/Basic/About-Actions.html). Every call
here is asynchronous: the run thread waits on the returned future while the
executor's other threads deliver the reply, so nothing spins twice.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
import math
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    PositionConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
import numpy as np
from numpy.typing import NDArray
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from ..core.planning import IKResult
from .tf_tools import pose_to_matrix

Mat = NDArray[np.float64]

ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
               if k.isupper() and isinstance(v, int)}
SUCCESS = MoveItErrorCodes.SUCCESS
TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)
GOAL_REPLY_TIMEOUT = 10.0   # s for the server to accept or reject a goal
MAX_OUTSTANDING_IK = 4      # timed-out IK calls still running on the server (§3.1)


def error_name(code: int) -> str:
    return ERROR_NAMES.get(code, f'error {code}')


def to_pose(T: Mat) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def wait_future(fut: Any, timeout: float,
                cancelled: threading.Event | Callable[[], bool] | None = None,
                poll: float = 0.01) -> bool:
    """True once ``fut`` is done; False at the deadline or once ``cancelled``
    (an Event, or a predicate) says stop."""
    deadline = time.monotonic() + timeout
    stop = cancelled.is_set if isinstance(cancelled, threading.Event) else cancelled
    while not fut.done():
        if time.monotonic() > deadline or (stop is not None and stop()):
            return False
        time.sleep(poll)
    return True


@dataclass
class GoalOutcome:
    """What one action goal did. ``stopped``: '' (ran to its end), 'cancel' or
    'timeout' (we cancelled it); ``stop_confirmed``: its terminal status arrived
    after our cancel (a cancel acknowledgement alone is not proof, §6)."""

    sent: bool = False             # the goal request went out
    answered: bool = False         # the server accepted or rejected it in time
    accepted: bool = False
    status: int = GoalStatus.STATUS_UNKNOWN
    error_code: int = 0
    planned_points: int = -1       # MoveGroup only: points in planned_trajectory
    planned: RobotTrajectory | None = None   # MoveGroup only: the planned trajectory
    stopped: str = ''
    stop_confirmed: bool = False
    note: str = ''


@dataclass
class CartesianPlan:
    ok: bool
    fraction: float = 0.0
    trajectory: RobotTrajectory | None = None
    note: str = ''


class MoveItClient:
    def __init__(self, node: Node, callback_group: Any) -> None:
        self.node = node
        cb = callback_group
        self.ik = node.create_client(GetPositionIK, 'compute_ik', callback_group=cb)
        self.cartesian = node.create_client(GetCartesianPath, 'compute_cartesian_path',
                                            callback_group=cb)
        self.fk = node.create_client(GetPositionFK, 'compute_fk', callback_group=cb)
        self.validity = node.create_client(GetStateValidity, 'check_state_validity',
                                           callback_group=cb)
        self.scene = node.create_client(GetPlanningScene, 'get_planning_scene',
                                        callback_group=cb)
        self.move = ActionClient(node, MoveGroup, 'move_action', callback_group=cb)
        self.execute = ActionClient(node, ExecuteTrajectory, 'execute_trajectory',
                                    callback_group=cb)
        self._lock = threading.Lock()
        self._active: list[tuple[Any, Any]] = []      # (goal handle, result future)
        self._late: list[Any] = []       # goal requests we gave up waiting for (§6)
        self._unconfirmed = 0            # goals that ended without a terminal status
        self._ik_pending: list[Any] = []

    # -- discovery ----------------------------------------------------------------
    def missing(self, timeout: float, cancelled: threading.Event | None = None) -> list[str]:
        """Names of the MoveIt interfaces not found within ``timeout``."""
        checks: list[tuple[str, Callable[[], bool]]] = [
            ('compute_ik', self.ik.service_is_ready),
            ('compute_cartesian_path', self.cartesian.service_is_ready),
            ('compute_fk', self.fk.service_is_ready),
            ('check_state_validity', self.validity.service_is_ready),
            ('move_action', self.move.server_is_ready),
            ('execute_trajectory', self.execute.server_is_ready)]
        deadline = time.monotonic() + timeout
        while True:
            missing = [n for n, ready in checks if not ready()]
            if not missing or time.monotonic() > deadline or \
                    (cancelled is not None and cancelled.is_set()):
                return missing
            time.sleep(0.05)

    # -- goals still running (the fault latch, §6) ------------------------------------
    def active(self) -> int:
        """Motion goals sent by this node that may still run: accepted goals whose
        result has not arrived, and goal requests we stopped waiting for (the
        server may still accept them; they are cancelled as soon as it does)."""
        with self._lock:
            for _h, f in self._active:
                if f.done() and int(f.result().status) not in TERMINAL:
                    self._unconfirmed += 1          # counted once, then dropped below
            self._active = [(h, f) for h, f in self._active if not f.done()]
            # a late request leaves _late only in its own callback, after its goal
            # (if accepted) is in _active: never a moment where it is in neither
            return len(self._active) + len(self._late)

    def unconfirmed(self) -> int:
        """Goals that ended without a terminal status (e.g. the server lost them):
        only measured stillness can clear them (``~/clear_fault``)."""
        self.active()
        with self._lock:
            return self._unconfirmed

    def forget_unconfirmed(self) -> None:
        with self._lock:
            self._unconfirmed = 0

    def _on_late_reply(self, fut: Any) -> None:
        """A goal request answered after we gave up: cancel it at once and track it."""
        try:
            handle = fut.result()
        except Exception:  # noqa: BLE001 - nothing to cancel
            handle = None
        if handle is not None and handle.accepted:
            res = handle.get_result_async()
            with self._lock:
                self._active.append((handle, res))
                self._late = [f for f in self._late if f is not fut]
            handle.cancel_goal_async()
            return
        with self._lock:
            self._late = [f for f in self._late if f is not fut]

    # -- IK (§3.1) ----------------------------------------------------------------------
    def solve_ik(self, base_T_ik: Mat, base_frame: str, group: str, ik_link: str,
                 joint_names: Sequence[str], ik_timeout: int,
                 cancelled: threading.Event | None = None) -> IKResult:
        """One candidate per request, on the monitored planning-scene state
        (``is_diff``, so attached bodies stay in the collision check), with
        collisions avoided. ``reason`` is the MoveIt error code name."""
        with self._lock:
            self._ik_pending = [f for f in self._ik_pending if not f.done()]
            if len(self._ik_pending) >= MAX_OUTSTANDING_IK:
                return IKResult(None, 'ik_backlog')      # the server is not keeping up
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name = group, ik_link
        r.robot_state.is_diff = True          # the scene's current state, attached bodies kept
        r.avoid_collisions = True
        r.pose_stamped.header.frame_id = base_frame
        r.pose_stamped.pose = to_pose(base_T_ik)
        r.timeout.sec = int(ik_timeout)        # whole seconds (Jazzy drops nanosec); 0 = default
        fut = self.ik.call_async(req)
        with self._lock:
            self._ik_pending.append(fut)
        # client deadline: the server's own timeout plus slack. It does not cancel the
        # server's work; the bound on outstanding calls above is what protects us.
        if not wait_future(fut, max(1, ik_timeout) + 5.0, cancelled):
            return IKResult(None, 'cancelled' if cancelled is not None and cancelled.is_set()
                            else 'ik_timeout')
        res = fut.result()
        code = int(res.error_code.val)
        if code != SUCCESS:
            return IKResult(None, error_name(code))
        js = res.solution.joint_state
        pos = dict(zip(js.name, js.position, strict=False))
        if not all(j in pos for j in joint_names):
            return IKResult(None, 'ik_solution_lacks_joints')
        return IKResult(np.array([pos[j] for j in joint_names], dtype=float))

    # -- motion (§3.2) --------------------------------------------------------------------
    def joint_goal(self, joints: Sequence[float], joint_names: Sequence[str], group: str,
                   base_frame: str, planning_time: float, velocity: float,
                   acceleration: float, path: Constraints | None = None) -> MoveGroup.Goal:
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = group
        r.num_planning_attempts = 1
        r.allowed_planning_time = float(planning_time)
        r.max_velocity_scaling_factor = float(velocity)
        r.max_acceleration_scaling_factor = float(acceleration)
        r.start_state.is_diff = True           # plan from the current monitored state
        r.workspace_parameters.header.frame_id = base_frame
        for c, v in ((r.workspace_parameters.min_corner, -2.0),
                     (r.workspace_parameters.max_corner, 2.0)):
            c.x = c.y = c.z = v
        goal = Constraints()
        for name, q in zip(joint_names, joints, strict=True):
            goal.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(q), tolerance_above=1e-4,
                tolerance_below=1e-4, weight=1.0))
        r.goal_constraints = [goal]
        if path is not None:
            r.path_constraints = path
        g.planning_options.plan_only = False
        g.planning_options.replan = False
        return g

    @staticmethod
    def box_constraint(ik_link: str, ik_T_tool: Mat, base_frame: str, centre: Mat,
                       size: Sequence[float]) -> Constraints:
        """The safety box as a path constraint on the tool origin (§3.2): a
        position constraint on the IK link with the tool offset as target point."""
        pc = PositionConstraint()
        pc.header.frame_id = base_frame
        pc.link_name = ik_link
        pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z = (
            float(v) for v in ik_T_tool[:3, 3])
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
        pc.constraint_region = BoundingVolume(primitives=[box], primitive_poses=[to_pose(centre)])
        pc.weight = 1.0
        return Constraints(name='handeye_autopilot_box', position_constraints=[pc])

    def plan_cartesian(self, waypoints_ik: Sequence[Mat], base_frame: str, group: str,
                       ik_link: str, max_step: float, jump: float, velocity: float,
                       acceleration: float, timeout: float,
                       cancelled: threading.Event | None = None) -> CartesianPlan:
        """The straight tool path through the converted waypoints (§3.2). Nothing
        moves here: a path MoveIt can only partly plan is refused before sending."""
        req = GetCartesianPath.Request()
        req.header.frame_id = base_frame
        req.start_state.is_diff = True
        req.group_name, req.link_name = group, ik_link
        req.waypoints = [to_pose(W) for W in waypoints_ik]
        req.max_step = float(max_step)
        req.avoid_collisions = True
        # the jump check (MoveIt's default 0 disables it); field names differ by
        # release, so set the ones this distro has
        if hasattr(req, 'revolute_jump_threshold'):
            req.revolute_jump_threshold = float(jump)
        elif hasattr(req, 'jump_threshold'):        # shortcut: older releases only have the
            req.jump_threshold = 0.0 if jump == 0 else 5.0   # relative factor
        for name, v in (('max_velocity_scaling_factor', velocity),
                        ('max_acceleration_scaling_factor', acceleration)):
            if hasattr(req, name):
                setattr(req, name, float(v))
        fut = self.cartesian.call_async(req)
        if not wait_future(fut, timeout, cancelled):
            return CartesianPlan(False, note='compute_cartesian_path did not reply')
        res = fut.result()
        code = int(res.error_code.val)
        frac = float(res.fraction)
        if code != SUCCESS or not math.isfinite(frac) or frac < 1.0 - 1e-9:
            return CartesianPlan(False, frac, note=f'Cartesian path {frac * 100:.0f} % planned '
                                 f'({error_name(code)})')
        # our own jump check: some releases accept the threshold field but do not
        # apply it, and a wrist branch flip between two path points is a large,
        # unchecked swing of the carried object
        pts = res.solution.joint_trajectory.points
        q = np.array([list(p.positions) for p in pts], dtype=float)
        if len(pts) < 1 or not np.all(np.isfinite(q)):
            return CartesianPlan(False, frac, note='Cartesian path with no or invalid points')
        if jump > 0 and len(q) > 1:
            worst = float(np.max(np.abs(np.diff(q, axis=0))))
            if worst > jump:
                return CartesianPlan(False, frac, note=f'Cartesian path has a joint jump of '
                                     f'{worst:.2f} rad between points (> {jump} rad)')
        return CartesianPlan(True, frac, res.solution)

    def fk_poses(self, names: Sequence[str], joints: Sequence[Sequence[float]], link: str,
                 base_frame: str, timeout: float,
                 cancelled: threading.Event | None = None) -> list[Mat] | None:
        """``link`` poses in ``base_frame`` for each joint vector (one request each)."""
        out = []
        for q in joints:
            req = GetPositionFK.Request()
            req.header.frame_id = base_frame
            req.fk_link_names = [link]
            req.robot_state.is_diff = True        # other joints: the current state
            req.robot_state.joint_state.name = list(names)
            req.robot_state.joint_state.position = [float(v) for v in q]
            fut = self.fk.call_async(req)
            if not wait_future(fut, timeout, cancelled):
                return None
            res = fut.result()
            if int(res.error_code.val) != SUCCESS or not res.pose_stamped:
                return None
            out.append(pose_to_matrix(res.pose_stamped[0].pose))
        return out

    def invalid_state(self, traj: RobotTrajectory, group: str, timeout: float,
                      cancelled: threading.Event | None = None) -> str:
        """Every trajectory point against the CURRENT planning scene (attached
        bodies included), right before execution: ExecuteTrajectory, unlike a
        MoveGroup goal, does not re-check a trajectory planned earlier. All
        requests go out at once, so the check spans one scene snapshot as far as
        possible (a few ms). '' if all are valid, else why."""
        jt = traj.joint_trajectory
        futs = []
        for p in jt.points:
            req = GetStateValidity.Request()
            req.group_name = group
            req.robot_state.is_diff = True
            req.robot_state.joint_state.name = list(jt.joint_names)
            req.robot_state.joint_state.position = [float(v) for v in p.positions]
            futs.append(self.validity.call_async(req))
        deadline = time.monotonic() + timeout
        for k, fut in enumerate(futs):
            if not wait_future(fut, max(0.0, deadline - time.monotonic()), cancelled):
                return 'check_state_validity did not reply'
            if not fut.result().valid:
                return f'trajectory point {k} is no longer valid in the current planning scene'
        return ''

    def attached_links(self, timeout: float) -> set[str] | None:
        """Links that carry attached collision objects in the planning scene, or
        None if the scene cannot be read."""
        if not self.scene.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        fut = self.scene.call_async(req)
        if not wait_future(fut, timeout):
            return None
        return {a.link_name for a in fut.result().scene.robot_state.attached_collision_objects}

    def execute_goal(self, trajectory: RobotTrajectory) -> ExecuteTrajectory.Goal:
        return ExecuteTrajectory.Goal(trajectory=trajectory)

    def run(self, client: ActionClient, goal: Any, timeout: float, cancel_timeout: float,
            cancelled: threading.Event, invalid: Callable[[], bool] | None = None,
            on_accepted: Callable[[], None] | None = None) -> GoalOutcome:
        """Send one goal and wait for its terminal status. A cancel (``cancelled``),
        the run becoming invalid (``invalid()``: a clock reset, a CameraInfo change)
        or the deadline cancels the goal and waits ``cancel_timeout`` for the
        terminal status that proves it stopped."""
        out = GoalOutcome(sent=True)
        send = client.send_goal_async(goal)
        wait_future(send, GOAL_REPLY_TIMEOUT)
        with self._lock:                  # decided once: a reply after this is 'late'
            late = not send.done()
            if late:
                self._late.append(send)
        if late:
            # the server may still accept it: then it is cancelled and tracked at once
            send.add_done_callback(self._on_late_reply)
            out.note = 'no reply to the goal request'
            return out
        out.answered = True
        handle = send.result()
        if not handle.accepted:
            out.note = 'goal rejected by the server'
            return out
        out.accepted = True
        res_fut = handle.get_result_async()
        with self._lock:
            self._active.append((handle, res_fut))
        if on_accepted is not None:
            on_accepted()
        if not wait_future(res_fut, timeout, lambda: cancelled.is_set() or
                           (invalid is not None and invalid())):
            out.stopped = 'cancel' if cancelled.is_set() else \
                'invalid' if invalid is not None and invalid() else 'timeout'
            handle.cancel_goal_async()
            out.stop_confirmed = wait_future(res_fut, cancel_timeout) and \
                int(res_fut.result().status) in TERMINAL
            if not out.stop_confirmed:
                out.note = f'no terminal status within {cancel_timeout:.1f} s of the cancel'
                return out
        res = res_fut.result()
        out.status = int(res.status)
        out.error_code = int(res.result.error_code.val)
        traj = getattr(res.result, 'planned_trajectory', None)
        if traj is not None:
            out.planned_points = len(traj.joint_trajectory.points)
            out.planned = traj
        return out
