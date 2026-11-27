"""Test harness for the M3 node: a fake robot TF publisher, scripted poses that
show the board, and an in-process driver for the action and services.

The fake robot publishes ``base -> tool`` on /tf at a fixed rate with the
current node-clock stamp, like robot_state_publisher; ``freeze`` stops it (a
frozen joint_states), ``delay`` delivers each transform late (delayed TF).
"""

from __future__ import annotations

from collections import deque
import math
import threading
import time

from handeye_autopilot.core import handeye
from handeye_autopilot.core.render import look_at_board
from handeye_autopilot.ros import tf_tools
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from handeye_autopilot_interfaces.action import Calibrate


class FakeRobot(Node):
    def __init__(self, base='base_link', tool='tool0', rate=50.0, **kw):
        super().__init__('fake_robot', **kw)
        self.base, self.tool = base, tool
        self.G = np.eye(4)
        self.freeze = False
        self.delay = 0.0
        self._queue: deque = deque()
        self.history: deque = deque(maxlen=5000)      # (stamp_ns, G) as published
        self._lock = threading.Lock()
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.create_timer(1.0 / rate, self._tick)

    def move_to(self, G):
        with self._lock:
            self.G = np.array(G, dtype=float)

    def _tick(self):
        now = self.get_clock().now()
        if not self.freeze:
            with self._lock:
                msg = tf_tools.to_msg(self.G, self.base, self.tool, now.to_msg())
                self.history.append((now.nanoseconds, self.G.copy()))
            self._queue.append((time.monotonic() + self.delay, msg))
        while self._queue and self._queue[0][0] <= time.monotonic():
            self.pub.publish(TFMessage(transforms=[self._queue.popleft()[1]]))


def truth(camera_node):
    """X (parent_T_optical) and Z from a SyntheticCamera instance."""
    return camera_node.true_X, camera_node.Z


def scripted_poses(mode, X, Z, camera, board, n=12, seed=1):
    """Tool poses that put the board in good, varied views (tilts, rolls, depths)."""
    rng = np.random.default_rng(seed)
    w, h = board.size_m
    centre = np.array([w / 2, h / 2, 0.0])
    out = []
    for _ in range(n):
        tx, ty = rng.uniform(-30, 30, 2)
        if abs(tx) + abs(ty) < 15:                 # never near face-on (ambiguity)
            tx = math.copysign(20, tx)
        C = look_at_board(centre, rng.uniform(0.5, 0.65), math.radians(tx),
                          math.radians(ty), math.radians(rng.uniform(-40, 40)),
                          offset_px=tuple(rng.uniform(-40, 40, 2)), camera=camera)
        out.append(handeye.tool_pose_for(mode, X, Z, C))
    return out


def seed_pose(mode, X, Z, camera, board, depth=0.65):
    """A MoveIt-mode seed: board centred, small enough in the image that the
    bootstrap's worst-case check allows useful steps (02-features §B2), tilted."""
    w, h = board.size_m
    C = look_at_board(np.array([w / 2, h / 2, 0.0]), depth, math.radians(20),
                      math.radians(-15), math.radians(5), offset_px=(0, 0), camera=camera)
    return handeye.tool_pose_for(mode, X, Z, C)


def camera_params(mode, **over):
    """SyntheticCamera parameters for a mode with sensible true mounts."""
    if mode == 'eye_to_hand':
        p = {'mode': mode, 'true_camera_xyz_rpy': [1.0, 0.1, 0.6, 0.05, 0.4, math.pi - 0.1],
             'true_board_xyz_rpy': [0.01, -0.02, 0.06, 0.05, -0.03, 0.2]}
    else:
        p = {'mode': mode, 'true_camera_xyz_rpy': [0.05, -0.03, 0.08, 0.1, -0.05, 0.3],
             'true_board_xyz_rpy': [0.6, 0.1, 0.0, 0.02, 0.03, 0.4]}
    p.update(over)
    return p


class Driver:
    """Action + service clients for the calibration node, waits with deadlines."""

    def __init__(self, node: Node, ns='/handeye_autopilot'):
        self.node = node
        self.action = ActionClient(node, Calibrate, f'{ns}/calibrate')
        self.capture = node.create_client(Trigger, f'{ns}/capture')
        self.finish = node.create_client(Trigger, f'{ns}/finish')
        self.clear_fault = node.create_client(Trigger, f'{ns}/clear_fault')
        self.inject_fault = node.create_client(Trigger, f'{ns}/inject_fault')
        self.cancel = node.create_client(Trigger, f'{ns}/cancel')
        self.feedback: list = []
        self.debug = None          # optional callable describing the node's state

    def wait(self, fut, timeout=30.0):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise TimeoutError('future not done')
            time.sleep(0.01)
        return fut.result()

    def send(self, mode='eye_to_hand', execution='manual', profile='', timeout=10.0):
        assert self.action.wait_for_server(timeout_sec=timeout)
        if not getattr(self, '_settled', False):
            # first contact: DDS discovery is per direction, and a goal reply sent
            # before the server has matched our reply reader is lost (seen on
            # loaded CI runners)
            time.sleep(1.0)
            self._settled = True
        g = Calibrate.Goal(mode=mode, execution=execution, profile=profile)
        return self.wait(self.action.send_goal_async(
            g, feedback_callback=lambda f: self.feedback.append(f.feedback)), timeout)

    def call(self, client, timeout=10.0):
        assert client.wait_for_service(timeout_sec=timeout)
        return self.wait(client.call_async(Trigger.Request()), timeout)

    def phase(self):
        return self.feedback[-1].phase if self.feedback else ''

    def wait_phase(self, phase, timeout=20.0):
        t0 = time.monotonic()
        while self.phase() != phase:
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(f'phase {phase!r} not reached (at {self.phase()!r})')
            time.sleep(0.02)

    def capture_and_wait(self, timeout=45.0):
        """Trigger one capture; returns the feedback after it finished."""
        n = len(self.feedback)
        res = self.call(self.capture)
        assert res.success, res.message
        t0 = time.monotonic()
        while True:
            seen = False
            for f in list(self.feedback[n:]):
                if f.phase == Calibrate.Feedback.PHASE_CAPTURING:
                    seen = True
                elif seen:
                    return f
            if time.monotonic() - t0 > timeout:
                phases = [f.phase for f in self.feedback[n:]]
                raise TimeoutError(f'capture did not finish; feedback since: {phases}; '
                                   f'state: {self.debug() if self.debug else None}')
            time.sleep(0.02)


class SimClock(Node):
    """Publishes /clock from wall time (ROS sim time); ``jump(dt)`` shifts it."""

    def __init__(self, start=1000.0, rate=200.0):
        super().__init__('sim_clock')
        from rosgraph_msgs.msg import Clock
        self._Clock = Clock
        self.offset = start - time.monotonic()
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.create_timer(1.0 / rate, self._tick)

    def jump(self, dt):
        self.offset += dt

    def pause(self):
        """Freeze sim time where it is (a paused simulator)."""
        self.frozen = time.monotonic() + self.offset

    def _tick(self):
        t = getattr(self, 'frozen', None) or time.monotonic() + self.offset
        m = self._Clock()
        m.clock.sec = int(t)
        m.clock.nanosec = int((t - int(t)) * 1e9)
        self.pub.publish(m)


# -- M4: a fake MoveIt (01-architecture §12: stub compute_ik, fake MoveGroup) -------

JOINTS = [f'j{i}' for i in range(1, 7)]


def encode(G):
    """Fake kinematics: joints = position + rotation vector of the IK-link pose."""
    from scipy.spatial.transform import Rotation
    return np.r_[G[:3, 3], Rotation.from_matrix(G[:3, :3]).as_rotvec()]


def decode(j):
    from handeye_autopilot.core import transforms as tf
    from scipy.spatial.transform import Rotation
    j = np.asarray(j, dtype=float)
    return tf.make(Rotation.from_rotvec(j[3:6]), j[:3])


class FakeMoveIt(Node):
    """compute_ik, compute_cartesian_path, move_action and execute_trajectory over
    a FakeRobot whose published frame is the IK link. Behaviour switches:

    - ``planning_only``: goals of the kinds in ``fault_kinds`` SUCCEED without
      moving the robot;
    - ``plan_fails(joints) -> bool | str``: True: that MoveGroup goal ABORTS with
      FAILURE and an empty planned trajectory, nothing moves; ``'invalid'``: it
      ABORTS with INVALID_MOTION_PLAN and the rejected plan filled in (MoveIt's
      ValidateSolution adapter), nothing moves; ``'invalid_moved'``: the same
      reply, but the robot moved to the goal first;
    - ``hang``: goals of the kinds in ``fault_kinds`` never reach a terminal state;
      cancels are accepted and ignored;
    - ``fault_kinds``: {'move', 'execute'} by default (MoveGroup / ExecuteTrajectory);
    - ``accept_delay``: seconds before a goal of a kind in ``fault_kinds`` is
      accepted (a slow server);
    - ``detour(n) -> bool``: the n-th plan-only MoveGroup plan passes through a
      point 1.5 m above the goal;
    - ``motion_time``: seconds a motion takes (cancellable unless ``hang``);
    - ``reach``: (r_min, r_max) of the IK-link origin from the base;
    - ``ik_collision(G) -> bool``: compute_ik answers NO_IK_SOLUTION (collision);
    - ``ik_delay``: seconds each compute_ik call takes.
    """

    def __init__(self, robot, **kw):
        super().__init__('fake_moveit', **kw)
        from moveit_msgs.action import ExecuteTrajectory, MoveGroup
        from moveit_msgs.srv import (
            GetCartesianPath,
            GetPlanningScene,
            GetPositionFK,
            GetPositionIK,
            GetStateValidity,
        )
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from sensor_msgs.msg import JointState
        self.robot = robot
        self.planning_only = False
        self.plan_fails = None
        self.hang = False
        self.fault_kinds = {'move', 'execute'}
        self.accept_delay = 0.0
        self.detour = None
        self.cart_jump = False           # Cartesian paths with a 1 rad joint jump
        self.cart_swing = False          # Cartesian paths that swing 5 cm off the line
                                         # in small steps (no single jump)
        self.invalid_states = False      # check_state_validity says invalid
        self.invalid_fn = None           # positions -> True if that state is invalid
        self.publish_joints = True       # False: /joint_states stops
        self.cart_fraction = None        # override the reported path fraction
        self.plans = 0                   # plan-only MoveGroup requests
        self.executed: list = []         # trajectories actually executed (joint lists)
        self.motion_time = 0.15
        self.reach = (0.05, 3.0)
        self.ik_collision = None
        self.ik_delay = 0.0              # s each compute_ik call takes
        self.goals: list = []            # (kind, request) per accepted goal
        self.ik_calls = 0
        self._release = threading.Event()
        cb = ReentrantCallbackGroup()
        self.create_service(GetPositionIK, 'compute_ik', self._ik, callback_group=cb)
        self.create_service(GetCartesianPath, 'compute_cartesian_path', self._cart,
                            callback_group=cb)
        self.create_service(GetPositionFK, 'compute_fk', self._fk, callback_group=cb)
        self.create_service(GetStateValidity, 'check_state_validity', self._valid,
                            callback_group=cb)
        self.create_service(GetPlanningScene, 'get_planning_scene', self._scene,
                            callback_group=cb)
        self.cancels: list = []          # kinds of goals whose cancel was requested

        def accept(handle):
            self.cancels.append(handle)
            return CancelResponse.ACCEPT

        def admit(kind):
            def cb_(_goal):
                if self.accept_delay and kind in self.fault_kinds:
                    time.sleep(self.accept_delay)
                return GoalResponse.ACCEPT
            return cb_
        self._move = ActionServer(self, MoveGroup, 'move_action', self._move_exec,
                                  goal_callback=admit('move'), cancel_callback=accept,
                                  callback_group=cb)
        self._exec = ActionServer(self, ExecuteTrajectory, 'execute_trajectory',
                                  self._traj_exec, goal_callback=admit('execute'),
                                  cancel_callback=accept, callback_group=cb)
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self._joints, callback_group=cb)
        self._JointState = JointState

    def release(self):
        self._release.set()

    def _joints(self):
        if not self.publish_joints:
            return
        m = self._JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(JOINTS)
        with self.robot._lock:
            m.position = [float(v) for v in encode(self.robot.G)]
        m.velocity = [0.0] * 6
        self.js.publish(m)

    def _reachable(self, G):
        r = float(np.linalg.norm(G[:3, 3]))
        return self.reach[0] <= r <= self.reach[1]

    def _ik(self, req, res):
        self.ik_calls += 1
        if self.ik_delay:
            time.sleep(self.ik_delay)
        G = tf_tools.pose_to_matrix(req.ik_request.pose_stamped.pose)
        if not self._reachable(G):
            res.error_code.val = -31                       # NO_IK_SOLUTION
        elif self.ik_collision is not None and self.ik_collision(G):
            res.error_code.val = -31
        else:
            res.error_code.val = 1
            res.solution.joint_state.name = list(JOINTS) + ['finger']
            res.solution.joint_state.position = [float(v) for v in encode(G)] + [0.0]
        return res

    def _valid(self, req, res):
        q = list(req.robot_state.joint_state.position)
        res.valid = not self.invalid_states and not (
            self.invalid_fn is not None and self.invalid_fn(q))
        return res

    def _scene(self, req, res):
        return res                       # no attached objects

    def _fk(self, req, res):
        js = req.robot_state.joint_state
        pos = dict(zip(js.name, js.position, strict=True))
        G = decode([pos[j] for j in JOINTS])
        p = tf_tools.to_msg(G, req.header.frame_id, req.fk_link_names[0])
        from geometry_msgs.msg import PoseStamped
        out = PoseStamped()
        out.header.frame_id = req.header.frame_id
        t = p.transform
        out.pose.position.x, out.pose.position.y, out.pose.position.z = (
            t.translation.x, t.translation.y, t.translation.z)
        out.pose.orientation = t.rotation
        res.pose_stamped = [out]
        res.fk_link_names = list(req.fk_link_names)
        res.error_code.val = 1
        return res

    def _cart(self, req, res):
        from trajectory_msgs.msg import JointTrajectoryPoint
        pts = []
        for p in req.waypoints:
            G = tf_tools.pose_to_matrix(p)
            if not self._reachable(G):
                break
            pts.append(JointTrajectoryPoint(positions=[float(v) for v in encode(G)]))
        if self.cart_jump and len(pts) > 1:
            pts[len(pts) // 2].positions[5] += 1.0
        if self.cart_swing and len(pts) > 1:
            mid = list(pts[len(pts) // 2].positions)
            bump = [JointTrajectoryPoint(positions=[float(v) for v in
                                                    np.r_[mid[:2], mid[2] + h, mid[3:]]])
                    for h in (0.01, 0.02, 0.03, 0.04, 0.05, 0.04, 0.03, 0.02, 0.01)]
            k = len(pts) // 2
            pts = pts[:k] + bump + pts[k:]
        res.fraction = len(pts) / max(1, len(req.waypoints))
        if self.cart_fraction is not None:
            res.fraction = self.cart_fraction
        res.solution.joint_trajectory.joint_names = list(JOINTS)
        res.solution.joint_trajectory.points = pts
        res.error_code.val = 1
        return res

    def _run_motion(self, goal_handle, target, result, kind):
        """Move the robot to ``target`` over motion_time; honour cancel/hang."""
        hang = self.hang and kind in self.fault_kinds
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.motion_time or hang:
            if self._release.is_set():
                break
            if goal_handle.is_cancel_requested and not hang:
                result.error_code.val = -7                 # PREEMPTED
                goal_handle.canceled(result)
                return result
            time.sleep(0.01)
        if self._release.is_set() and hang:
            goal_handle.abort(result)
            return result
        if target is not None and not (self.planning_only and kind in self.fault_kinds):
            self.robot.move_to(target)
        result.error_code.val = 1
        goal_handle.succeed(result)
        return result

    def _move_exec(self, goal_handle):
        from moveit_msgs.action import MoveGroup
        from trajectory_msgs.msg import JointTrajectoryPoint
        req = goal_handle.request.request
        self.goals.append(('move', req))
        res = MoveGroup.Result()
        c = req.goal_constraints[0].joint_constraints
        joints = [jc.position for jc in sorted(c, key=lambda jc: JOINTS.index(jc.joint_name))]
        fail = self.plan_fails(joints) if self.plan_fails is not None else False
        if fail is True:
            res.error_code.val = 99999                     # FAILURE, nothing planned
            goal_handle.abort(res)
            return res
        start = list(encode(self.robot.G))
        pts = [start, list(joints)]
        if fail in ('invalid', 'invalid_moved'):
            res.planned_trajectory.joint_trajectory.joint_names = list(JOINTS)
            res.planned_trajectory.joint_trajectory.points = [
                JointTrajectoryPoint(positions=[float(v) for v in p]) for p in pts]
            if fail == 'invalid_moved':
                self.robot.move_to(decode(joints))
                time.sleep(0.3)
            res.error_code.val = -2                        # INVALID_MOTION_PLAN
            goal_handle.abort(res)
            return res
        if goal_handle.request.planning_options.plan_only:
            self.plans += 1
            if self.detour is not None and self.detour(self.plans):
                up = list(joints)
                up[2] += 1.5                               # 1.5 m above the goal
                pts = [start, up, list(joints)]
        res.planned_trajectory.joint_trajectory.joint_names = list(JOINTS)
        res.planned_trajectory.joint_trajectory.points = [
            JointTrajectoryPoint(positions=[float(v) for v in p]) for p in pts]
        if goal_handle.request.planning_options.plan_only:
            res.error_code.val = 1
            goal_handle.succeed(res)
            return res
        return self._run_motion(goal_handle, decode(joints), res, 'move')

    def _traj_exec(self, goal_handle):
        from moveit_msgs.action import ExecuteTrajectory
        traj = goal_handle.request.trajectory.joint_trajectory
        self.goals.append(('execute', traj))
        self.executed.append([list(p.positions) for p in traj.points])
        res = ExecuteTrajectory.Result()
        target = decode(traj.points[-1].positions) if traj.points else None
        return self._run_motion(goal_handle, target, res, 'execute')
