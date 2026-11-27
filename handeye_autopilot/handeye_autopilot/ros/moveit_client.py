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
    PositionConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
import numpy as np
from numpy.typing import NDArray
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive

from ..core.planning import IKResult

Mat = NDArray[np.float64]

ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
               if k.isupper() and isinstance(v, int)}
SUCCESS = MoveItErrorCodes.SUCCESS
MAX_OUTSTANDING_IK = 4      # timed-out IK calls still running on the server (§3.1)


def error_name(code: int) -> str:
    return ERROR_NAMES.get(code, f'error {code}')


def to_pose(T: Mat) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def wait_future(fut: Any, timeout: float, cancelled: threading.Event | None = None,
                poll: float = 0.01) -> bool:
    """True once ``fut`` is done; False at the deadline or on cancel."""
    deadline = time.monotonic() + timeout
    while not fut.done():
        if time.monotonic() > deadline or (cancelled is not None and cancelled.is_set()):
            return False
        time.sleep(poll)
    return True


@dataclass
class GoalOutcome:
    """What one action goal did. ``stopped``: '' (ran to its end), 'cancel' or
    'timeout' (we cancelled it); ``stop_confirmed``: its terminal status arrived
    after our cancel (a cancel acknowledgement alone is not proof, §6)."""

    sent: bool = False             # the goal request went out
    accepted: bool = False
    status: int = GoalStatus.STATUS_UNKNOWN
    error_code: int = 0
    planned_points: int = -1       # MoveGroup only: points in planned_trajectory
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
        self.move = ActionClient(node, MoveGroup, 'move_action', callback_group=cb)
        self.execute = ActionClient(node, ExecuteTrajectory, 'execute_trajectory',
                                    callback_group=cb)
        self._lock = threading.Lock()
        self._active: list[tuple[Any, Any]] = []      # (goal handle, result future)
        self._ik_pending: list[Any] = []

    # -- discovery ----------------------------------------------------------------
    def missing(self, timeout: float, cancelled: threading.Event | None = None) -> list[str]:
        """Names of the MoveIt interfaces not found within ``timeout``."""
        checks: list[tuple[str, Callable[[], bool]]] = [
            ('compute_ik', self.ik.service_is_ready),
            ('compute_cartesian_path', self.cartesian.service_is_ready),
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
        """Motion goals sent by this node whose terminal status has not arrived."""
        with self._lock:
            self._active = [(h, f) for h, f in self._active if not f.done()]
            return len(self._active)

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
        if code != SUCCESS or frac < 1.0 - 1e-9:
            return CartesianPlan(False, frac, note=f'Cartesian path {frac * 100:.0f} % planned '
                                 f'({error_name(code)})')
        return CartesianPlan(True, frac, res.solution)

    def execute_goal(self, trajectory: RobotTrajectory) -> ExecuteTrajectory.Goal:
        return ExecuteTrajectory.Goal(trajectory=trajectory)

    def run(self, client: ActionClient, goal: Any, timeout: float, cancel_timeout: float,
            cancelled: threading.Event, on_accepted: Callable[[], None] | None = None
            ) -> GoalOutcome:
        """Send one goal and wait for its terminal status. A cancel (``cancelled``)
        or the deadline cancels the goal and waits ``cancel_timeout`` for the
        terminal status that proves it stopped."""
        out = GoalOutcome(sent=True)
        send = client.send_goal_async(goal)
        if not wait_future(send, 10.0):
            # the goal may be running on the server with no handle to cancel it
            out.note = 'no reply to the goal request'
            return out
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
        if not wait_future(res_fut, timeout, cancelled):
            out.stopped = 'cancel' if cancelled.is_set() else 'timeout'
            handle.cancel_goal_async()
            out.stop_confirmed = wait_future(res_fut, cancel_timeout)
            if not out.stop_confirmed:
                out.note = f'no terminal status within {cancel_timeout:.1f} s of the cancel'
                return out
        res = res_fut.result()
        out.status = int(res.status)
        out.error_code = int(res.result.error_code.val)
        traj = getattr(res.result, 'planned_trajectory', None)
        if traj is not None:
            out.planned_points = len(traj.joint_trajectory.points)
        return out
