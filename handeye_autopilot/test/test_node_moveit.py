"""M4 node tests with a fake MoveIt, in one process (03-milestones M4): full
MoveIt-mode calibrations in both modes (one with a TF-only TCP), a planning
failure taken as a safe skip, and the failure tests required before hardware:
a planning-only move_group reporting success without motion (caught by the TF
check), a cancel whose goal never reaches a terminal state (MOTION_FAULT, the
latch, ~/clear_fault), a cancel during motion, and the missing IK service.
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from handeye_autopilot.core import transforms as tf  # noqa: E402
from handeye_autopilot.ros import tf_tools  # noqa: E402
from ros_harness import JOINTS, FakeMoveIt, seed_pose  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from test_node_manual import World  # noqa: E402
import tf2_ros  # noqa: E402

from handeye_autopilot_interfaces.action import Calibrate  # noqa: E402

R = Calibrate.Result
FB = Calibrate.Feedback

# own DDS domain per test process (see the ros fixture): two test runs at once
# must not answer each other's fake MoveIt
DOMAIN = str(30 + os.getpid() % 60)

MOVEIT = {'moveit.group_name': 'arm', 'moveit.joint_names': JOINTS,
          'moveit.service_timeout': 2.0, 'moveit.cancel_timeout': 1.0,
          'moveit.motion_timeout': 20.0,
          'session.lever_max': 0.15}      # the true lever arms are ~0.1 m (camera_params)


class MoveItWorld(World):
    """World + FakeMoveIt. ``tcp_offset``: the robot publishes ``flange`` (the IK
    link) and a static ``flange -> tool0`` (a TCP known only to TF)."""

    def __init__(self, mode, tmp_path, node_over=None, tcp_offset=None, fake=True, **kw):
        self.flange_T_tool = None if tcp_offset is None else tf.make(
            Rotation.from_euler('xyz', tcp_offset[3:]), np.asarray(tcp_offset[:3], float))
        over = dict(MOVEIT, **(node_over or {}))
        if tcp_offset is not None:
            over['moveit.ik_link'] = 'flange'
        super().__init__(mode, tmp_path, node_over=over, **kw)
        if tcp_offset is not None:
            self.robot.tool = 'flange'
            self.static = tf2_ros.StaticTransformBroadcaster(self.robot)
            self.static.sendTransform(tf_tools.to_msg(self.flange_T_tool, 'flange', 'tool0'))
        self.moveit = FakeMoveIt(self.robot) if fake else None
        if self.moveit is not None:
            self.ex.add_node(self.moveit)
            self.nodes.append(self.moveit)
        self.place(seed_pose(mode, self.X, self.Z, self.camera.camera, self.camera.board))
        time.sleep(1.0)

    def place(self, G_tool):
        """Put the TOOL at G_tool (the robot publishes the IK link)."""
        if self.flange_T_tool is None:
            self.robot.move_to(G_tool)
        else:
            self.robot.move_to(G_tool @ tf.inv(self.flange_T_tool))

    def close(self):
        if self.moveit is not None:
            self.moveit.release()
        super().close()


@pytest.fixture(scope='module', autouse=True)
def ros():
    # own DDS domain: the fake serves compute_ik / move_action under MoveIt's
    # names, and a real move_group (the demos' tests, run in parallel by colcon)
    # must not answer it
    old = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    rclpy.init()
    yield
    rclpy.shutdown()
    if old is None:
        del os.environ['ROS_DOMAIN_ID']
    else:
        os.environ['ROS_DOMAIN_ID'] = old


@pytest.fixture
def moveit_world(tmp_path):
    worlds = []

    def make(mode='eye_to_hand', **kw):
        w = MoveItWorld(mode, tmp_path, **kw)
        worlds.append(w)
        return w
    yield make
    for w in worlds:
        w.close()


def _x(res):
    t, q = res.camera_transform.transform.translation, res.camera_transform.transform.rotation
    return tf.make(Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z]))


def _wait(pred, timeout, what):
    t0 = time.monotonic()
    while not pred():
        if time.monotonic() - t0 > timeout:
            raise TimeoutError(what)
        time.sleep(0.02)


@pytest.mark.parametrize('mode,tcp', [('eye_to_hand', None),
                                      ('eye_in_hand', [0.0, 0.0, 0.1, 0.0, 0.0, 0.6])])
def test_moveit_calibration_recovers_the_true_mount(moveit_world, mode, tcp):
    w = moveit_world(mode, tcp_offset=tcp)
    fails = {'n': 0}
    # eye-in-hand: MoveIt's own ValidateSolution rejects the plan (INVALID_MOTION_PLAN,
    # plan filled in, nothing moved; seen in the mount sweep), also a safe skip
    kind = True if mode == 'eye_to_hand' else 'invalid'

    def plan_fails(joints):               # the first two planned moves cannot be planned:
        if fails['n'] < 2:                # safe skips, spares take their place
            fails['n'] += 1
            return kind
        return False
    w.moveit.plan_fails = plan_fails
    h = w.driver.send(mode=mode, execution='moveit')
    assert h.accepted
    out = w.result(h, timeout=400.0)
    res = out.result
    assert out.status == 4, res.message                       # SUCCEEDED
    assert res.error_code == R.NONE, res.message
    assert tf.translation_distance(_x(res), w.X) < 0.003, res.message
    assert math.degrees(tf.rotation_angle(_x(res), w.X)) < 0.5
    phases = {f.phase for f in w.driver.feedback}
    for p in (FB.PHASE_PREFLIGHT, FB.PHASE_SEED_CHECK, FB.PHASE_BOOTSTRAP, FB.PHASE_CENTRING,
              FB.PHASE_PLANNING, FB.PHASE_MOVING, FB.PHASE_CAPTURING, FB.PHASE_SOLVING,
              FB.PHASE_VERIFYING):
        assert p in phases, (p, phases)
    kinds = {k for k, _ in w.moveit.goals}
    assert kinds == {'move', 'execute'}          # planned joint goals + Cartesian checked moves
    assert fails['n'] == 2 and res.passed, res.message
    if tcp is not None:
        # Cartesian waypoints went to MoveIt as IK-link (flange) poses
        _, traj = next(g for g in w.moveit.goals if g[0] == 'execute')
        assert traj.points


@pytest.mark.parametrize('kind', ['execute', 'move'])
def test_planning_only_success_is_caught_by_the_tf_check(moveit_world, kind):
    """03-milestones M4: a move_group that reports success without moving, for
    Cartesian moves (ExecuteTrajectory, bootstrap) and planned joint goals
    (MoveGroup, after bootstrap and centring)."""
    w = moveit_world()
    w.moveit.planning_only = True
    w.moveit.fault_kinds = {kind}
    h = w.driver.send(execution='moveit')
    assert h.accepted
    out = w.result(h, timeout=300.0)
    assert out.status == 6                                    # ABORTED
    assert out.result.error_code == R.MOTION_FAILED, out.result.message
    assert 'measured tool pose' in out.result.message
    assert w.moveit.goals[-1][0] == kind                      # the faulty action was hit


@pytest.mark.parametrize('kind', ['invalid_moved', 'invalid_roundtrip'])
def test_invalid_plan_after_moving_is_a_failure_not_a_skip(moveit_world, kind):
    """INVALID_MOTION_PLAN is a safe skip only if the joints were watched and
    never left the start while the goal was active: the same reply after the
    robot moved, even if it came back exactly, ends the run (MOTION_FAILED)."""
    w = moveit_world()
    w.moveit.plan_fails = lambda joints: kind
    h = w.driver.send(execution='moveit')
    assert h.accepted
    out = w.result(h, timeout=300.0)
    assert out.status == 6                                    # ABORTED
    assert out.result.error_code == R.MOTION_FAILED, out.result.message
    assert 'INVALID_MOTION_PLAN' in out.result.message
    if kind == 'invalid_roundtrip':      # caught by the watch seeing the motion, not a gap
        assert 'a sample left the start' in out.result.message, out.result.message
    assert w.moveit.goals[-1][0] == 'move'


@pytest.mark.parametrize('kind', ['execute', 'move'])
def test_cancel_without_terminal_state_latches_a_motion_fault(moveit_world, kind):
    """03-milestones M4: the MoveGroup (or ExecuteTrajectory) goal never
    terminates after the cancel -> ABORTED / MOTION_FAULT; new goals are
    rejected until ~/clear_fault succeeds."""
    w = moveit_world()
    w.moveit.hang = True
    w.moveit.fault_kinds = {kind}
    d = w.driver
    h = d.send(execution='moveit')
    assert h.accepted
    _wait(lambda: any(k == kind for k, _ in w.moveit.goals), 300.0, f'no {kind} goal sent')
    time.sleep(0.3)
    d.wait(h.cancel_goal_async(), 10.0)
    out = w.result(h, timeout=30.0)
    assert out.status == 6, out.result.message               # ABORTED, not CANCELED
    assert out.result.error_code == R.MOTION_FAULT, out.result.message
    assert w.moveit.goals[-1][0] == kind
    assert not d.send(execution='moveit').accepted            # latched
    res = d.call(d.clear_fault)
    assert not res.success and 'still active' in res.message
    w.moveit.release()                                        # the goal finally ends
    _wait(lambda: not w.node.moveit.active(), 10.0, 'goal still active')
    res = d.call(d.clear_fault)
    assert res.success, res.message
    w.moveit.hang = False
    h2 = d.send(execution='moveit')
    assert h2.accepted
    d.wait(h2.cancel_goal_async(), 10.0)
    w.result(h2, timeout=60.0)


def test_cancel_during_motion_is_canceled(moveit_world):
    w = moveit_world()
    w.moveit.motion_time = 5.0
    d = w.driver
    h = d.send(execution='moveit')
    assert h.accepted
    _wait(lambda: w.moveit.goals, 120.0, 'no motion goal sent')
    time.sleep(0.3)
    d.wait(h.cancel_goal_async(), 10.0)
    out = w.result(h, timeout=30.0)
    assert out.status == 5, out.result.message               # CANCELED
    assert out.result.error_code == R.CANCELED
    assert d.send(execution='moveit').accepted is True        # no latch
    # (the second goal is cancelled by the fixture's close)


def test_late_goal_reply_is_a_fault_and_the_goal_is_cancelled(moveit_world, monkeypatch):
    """A goal the server accepts only after we stopped waiting may run with no
    handle: MOTION_FAULT, and the late goal is cancelled and tracked, so
    ~/clear_fault refuses while it is active."""
    from handeye_autopilot.ros import moveit_client
    monkeypatch.setattr(moveit_client, 'GOAL_REPLY_TIMEOUT', 1.0)
    w = moveit_world()
    w.moveit.accept_delay = 3.0
    w.moveit.motion_time = 6.0
    w.moveit.fault_kinds = {'execute'}
    d = w.driver
    h = d.send(execution='moveit')
    out = w.result(h, timeout=120.0)
    assert out.result.error_code == R.MOTION_FAULT, out.result.message
    _wait(lambda: w.node.moveit.active(), 10.0, 'late goal not tracked')
    res = d.call(d.clear_fault)
    assert not res.success and 'active' in res.message
    _wait(lambda: not w.node.moveit.active(), 15.0, 'late goal not cancelled')
    res = d.call(d.clear_fault)
    assert res.success, res.message


def test_box_guards_the_carried_object_on_planned_paths(moveit_world):
    """01-architecture §3.2: with box_check_moving_object, a planned move is
    planned only, its path is checked (L_max ball inside the box via FK) and
    only then executed. Plans that leave the box are never executed (a spare
    replaces them); the run still succeeds."""
    w = moveit_world(node_over={'session.box_enabled': True,
                                'session.box_check_moving_object': True})
    # a box around the work area (seed tool +- 1 m), far smaller than the 1.5 m detour
    G0 = w.robot.G
    w.node.set_parameters([rclpy.parameter.Parameter(
        'session.box_xyz', value=[float(v) for v in G0[:3, 3]]),
        rclpy.parameter.Parameter('session.box_size', value=[2.0, 2.0, 2.0])])
    w.moveit.detour = lambda n: n <= 2
    h = w.driver.send(execution='moveit')
    out = w.result(h, timeout=400.0)
    assert out.result.error_code == R.NONE, out.result.message
    assert w.moveit.plans >= 3
    detoured = [t for t in w.moveit.executed if len(t) == 3 and
                t[1][2] - t[2][2] > 1.0]
    assert not detoured, 'a plan leaving the box was executed'
    planned_and_run = [t for t in w.moveit.executed if len(t) == 2]
    assert planned_and_run, 'no checked plan was executed'


@pytest.mark.parametrize('fault', ['jump', 'nan_fraction', 'swing', 'invalid_state'])
def test_bad_cartesian_paths_are_never_executed(moveit_world, fault):
    """Refused before anything is sent (not_started, nothing runs): a path with a
    joint jump (some releases ignore the threshold field), a NaN fraction, a
    path that swings off the straight tool line in steps too small for any jump
    check (a resampled branch flip), and a path no longer valid in the current
    planning scene."""
    w = moveit_world()
    if fault == 'jump':
        w.moveit.cart_jump = True
    elif fault == 'swing':
        w.moveit.cart_swing = True
    elif fault == 'invalid_state':
        w.moveit.invalid_states = True
    else:
        w.moveit.cart_fraction = float('nan')
    out = w.result(w.driver.send(execution='moveit'), timeout=120.0)
    assert out.result.error_code == R.NOT_ENOUGH_SAMPLES, out.result.message
    assert 'bootstrap_failed' in out.result.message
    assert not [k for k, _ in w.moveit.goals if k == 'execute']


@pytest.mark.parametrize('limit', ['box', 'envelope'])
def test_cartesian_path_is_limit_checked_as_planned(moveit_world, limit):
    """The core checks the nominal waypoints; the path MoveIt plans may deviate
    within the straight-path tolerance. Here the tolerance is loose and the
    planned path swings 5 cm up, through the box top or out of the seed
    envelope: refused, nothing runs."""
    w = moveit_world(node_over={'moveit.pose_tolerance_translation': 0.2})
    G0 = w.robot.G
    if limit == 'box':
        w.node.set_parameters([
            rclpy.parameter.Parameter('session.box_enabled', value=True),
            rclpy.parameter.Parameter('session.box_xyz',
                                      value=[float(v) for v in G0[:3, 3]]),
            rclpy.parameter.Parameter('session.box_size', value=[2.0, 2.0, 0.06])])
    else:
        w.node.set_parameters([rclpy.parameter.Parameter('session.envelope_radius',
                                                         value=0.035)])
    w.moveit.cart_swing = True
    out = w.result(w.driver.send(execution='moveit'), timeout=120.0)
    assert out.result.error_code == R.NOT_ENOUGH_SAMPLES, out.result.message
    assert not [k for k, _ in w.moveit.goals if k == 'execute']


def test_ok_needs_fresh_joints_after_the_motion(moveit_world):
    """§3.2: 'ok' carries fresh measured joints; if /joint_states stops, the
    motion's end state is unknown: MOTION_FAILED."""
    w = moveit_world()
    w.moveit.motion_time = 2.0
    h = w.driver.send(execution='moveit')
    _wait(lambda: w.moveit.goals, 120.0, 'no motion goal')
    w.moveit.publish_joints = False
    out = w.result(h, timeout=60.0)
    assert out.result.error_code == R.MOTION_FAILED, out.result.message
    assert 'joint_states' in out.result.message


def test_cancel_service_stops_a_run_without_its_goal_handle(moveit_world):
    """~/cancel stops the active run from anywhere (an action client whose goal
    reply was lost has no handle): the motion goal is cancelled and the goal
    ends ABORTED with error_code CANCELED (the action protocol allows CANCELED
    only after an action cancel request)."""
    w = moveit_world()
    w.moveit.motion_time = 6.0
    d = w.driver
    assert not d.call(d.cancel).success                  # nothing to cancel yet
    h = d.send(execution='moveit')
    _wait(lambda: w.moveit.goals, 120.0, 'no motion goal')
    time.sleep(0.3)
    t0 = time.monotonic()
    assert d.call(d.cancel).success
    out = w.result(h, timeout=30.0)
    assert time.monotonic() - t0 < 4.0
    assert out.status == 6 and out.result.error_code == R.CANCELED, out.result.message
    assert w.moveit.cancels


def test_validity_is_checked_between_trajectory_points(moveit_world):
    """The scene re-check covers the path between returned points: a state
    that is invalid only between two valid points is found."""
    from handeye_autopilot.ros.moveit_client import MoveItClient
    from moveit_msgs.msg import RobotTrajectory
    from rclpy.callback_groups import ReentrantCallbackGroup
    from trajectory_msgs.msg import JointTrajectoryPoint
    w = moveit_world()
    client = MoveItClient(w.client, ReentrantCallbackGroup())
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = list(JOINTS)
    traj.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[0.0, 0.0, 0.5, 0.0, 0.0, 0.0]),
        JointTrajectoryPoint(positions=[0.2, 0.0, 0.5, 0.0, 0.0, 0.0])]
    w.moveit.invalid_fn = lambda q: 0.09 < q[0] < 0.11      # an obstacle midway
    assert 'no longer valid' in client.invalid_state(traj, 'arm', 10.0)
    w.moveit.invalid_fn = lambda q: 0.3 < q[0]               # off the path
    assert client.invalid_state(traj, 'arm', 10.0) == ''


def test_camera_change_during_a_motion_stops_it(moveit_world):
    """§6-7: the run becomes invalid while a goal runs: the goal is cancelled at
    once (not only noticed after it ends) and the run ends NO_CAMERA_INFO."""
    from handeye_autopilot.core.camera import CameraModel
    w = moveit_world()
    w.moveit.motion_time = 6.0
    h = w.driver.send(execution='moveit')
    _wait(lambda: w.moveit.goals, 120.0, 'no motion goal')
    time.sleep(0.5)
    c = w.camera.camera
    w.camera.camera = CameraModel(c.K * np.array([[1.01, 1, 1], [1, 1.01, 1], [1, 1, 1]]),
                                  c.D, c.width, c.height)
    t0 = time.monotonic()
    out = w.result(h, timeout=30.0)
    assert out.result.error_code == R.NO_CAMERA_INFO, out.result.message
    assert time.monotonic() - t0 < 4.0                  # before the 6 s motion ended
    assert w.moveit.cancels


def test_box_in_a_named_frame_is_resolved_through_tf(moveit_world):
    """§3.4: session.box_frame: the box is posed in that frame (here 1 m above
    the base, so the published box sits at z + 1)."""
    from visualization_msgs.msg import MarkerArray
    w = moveit_world(node_over={'session.box_enabled': True, 'session.box_frame': 'shelf',
                                'session.box_xyz': [0.1, 0.0, 0.2],
                                'session.box_size': [3.0, 3.0, 3.0]})
    st = tf2_ros.StaticTransformBroadcaster(w.robot)
    st.sendTransform(tf_tools.to_msg(tf.make(np.eye(3), np.array([0.0, 0.0, 1.0])),
                                     'base_link', 'shelf'))
    got = []
    w.client.create_subscription(MarkerArray, '/handeye_autopilot/markers',
                                 lambda m: got.extend(x for x in m.markers
                                                      if x.ns == 'safety_box'), 10)
    time.sleep(0.5)
    h = w.driver.send(execution='manual')
    _wait(lambda: got, 10.0, 'no safety box marker')
    p = got[-1].pose.position
    assert got[-1].header.frame_id == 'base_link'
    assert abs(p.x - 0.1) < 1e-6 and abs(p.z - 1.2) < 1e-6
    w.driver.wait(h.cancel_goal_async(), 10.0)


def test_missing_box_frame_aborts(moveit_world):
    w = moveit_world(node_over={'session.box_enabled': True, 'session.box_frame': 'nowhere'})
    out = w.result(w.driver.send(execution='moveit'), timeout=30.0)
    assert out.result.error_code == R.TF_UNAVAILABLE and 'nowhere' in out.result.message


def test_carried_geometry_helpers():
    """§3.4 warning: links rigid below the tool, and whether they collide."""
    from handeye_autopilot.ros.node import _fixed_descendants, _has_collision
    urdf = '''<robot name="r">
      <link name="flange"/><link name="tool0"/>
      <link name="board"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
      <link name="finger"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
      <joint name="a" type="fixed"><parent link="flange"/><child link="tool0"/></joint>
      <joint name="b" type="fixed"><parent link="tool0"/><child link="board"/></joint>
      <joint name="c" type="prismatic"><parent link="tool0"/><child link="finger"/></joint>
    </robot>'''
    assert _fixed_descendants(urdf, {'tool0'}) == {'board'}
    assert _has_collision(urdf, 'board') and not _has_collision(urdf, 'tool0')
    assert _fixed_descendants('', {'tool0'}) == set()


def test_fault_clears_with_a_500hz_joint_stream(moveit_world):
    """A UR driver publishes joint states at 500 Hz: the history must still span
    the 0.5 s stillness window."""
    from sensor_msgs.msg import JointState
    w = moveit_world(fake=False)
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    d = w.driver
    assert d.call(d.inject_fault).success
    stop = threading.Event()

    def stream():
        while not stop.is_set():
            m = JointState(name=list(JOINTS), position=[0.1] * 6, velocity=[0.0] * 6)
            m.header.stamp = w.node.get_clock().now().to_msg()
            pub.publish(m)
            time.sleep(0.002)
    threading.Thread(target=stream, daemon=True).start()
    try:
        res = d.call(d.clear_fault, timeout=15.0)
    finally:
        stop.set()
    assert res.success, res.message


def test_joint_evidence_needs_fresh_finite_stamps(moveit_world):
    """Replayed (old-stamped) or NaN joint states are not measurements."""
    from sensor_msgs.msg import JointState
    w = moveit_world(fake=False)
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    run = type('Run', (), {'cancelled': threading.Event()})()

    def send(stamp_offset, value):
        m = JointState(name=list(JOINTS), position=[value] * 6)
        m.header.stamp = tf_tools.stamp_to_time(w.node.now() + stamp_offset).to_msg()
        pub.publish(m)
    t0 = time.monotonic()
    threading.Timer(0.3, send, (-30.0, 0.1)).start()          # a replay from the past
    assert w.node._joints_now(JOINTS, t0, 1.0, run) is None
    t0 = time.monotonic()
    threading.Timer(0.3, send, (0.0, float('nan'))).start()
    assert w.node._joints_now(JOINTS, t0, 1.0, run) is None
    t0 = time.monotonic()
    threading.Timer(0.3, send, (0.0, 0.2)).start()
    q = w.node._joints_now(JOINTS, t0, 2.0, run)
    assert q is not None and np.allclose(q, 0.2)


def test_clear_fault_needs_every_joint_ever_seen(moveit_world):
    """The arm's stream stopped and only a gripper keeps publishing: the fault
    must not clear on the gripper's stillness alone."""
    from sensor_msgs.msg import JointState
    w = moveit_world(fake=False)
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    d = w.driver
    assert d.call(d.inject_fault).success
    pub.publish(JointState(name=list(JOINTS), position=[0.0] * 6))      # the arm, once
    time.sleep(0.3)
    stop = threading.Event()

    def gripper():
        while not stop.is_set():
            m = JointState(name=['finger'], position=[0.0], velocity=[0.0])
            m.header.stamp = w.node.get_clock().now().to_msg()
            for _ in range(250):          # flood: the arm message leaves the buffer
                pub.publish(m)
            time.sleep(0.05)
    threading.Thread(target=gripper, daemon=True).start()
    try:
        res = d.call(d.clear_fault, timeout=15.0)
    finally:
        stop.set()
    assert not res.success and 'j1' in res.message


def test_cancel_during_ik_is_prompt(moveit_world):
    """§6: a cancel while compute_ik is computing stops issuing requests and
    drops the outstanding reply."""
    w = moveit_world()
    w.moveit.ik_delay = 4.0
    d = w.driver
    h = d.send(execution='moveit')
    assert h.accepted
    _wait(lambda: w.moveit.ik_calls > 0, 60.0, 'no IK request')
    time.sleep(0.3)
    t0 = time.monotonic()
    d.wait(h.cancel_goal_async(), 10.0)
    out = w.result(h, timeout=30.0)
    assert out.status == 5, out.result.message               # CANCELED
    assert time.monotonic() - t0 < 3.0                        # not after the IK reply
    assert w.moveit.ik_calls == 1


def test_missing_ik_service_aborts(moveit_world):
    w = moveit_world(fake=False, node_over={'moveit.service_timeout': 1.0})
    h = w.driver.send(execution='moveit')
    out = w.result(h, timeout=60.0)
    assert out.result.error_code == R.IK_SERVICE_MISSING, out.result.message


def test_moveit_mode_needs_the_group(moveit_world):
    w = moveit_world(node_over={'moveit.group_name': ''})
    out = w.result(w.driver.send(execution='moveit'), timeout=30.0)
    assert out.result.error_code == R.INVALID_GOAL
    assert 'moveit.group_name' in out.result.message


def test_moving_tcp_offset_is_refused(moveit_world):
    """§3.1: the ik_link -> tool transform must be fixed (here it moves on /tf)."""
    w = moveit_world(node_over={'moveit.ik_link': 'flange'})
    stop = threading.Event()
    pub = tf2_ros.TransformBroadcaster(w.robot)

    def wiggle():
        k = 0
        while not stop.is_set():
            k += 1
            T = tf.make(np.eye(3), np.array([0.0, 0.0, 0.1 + 0.001 * k]))   # ever-moving
            pub.sendTransform(tf_tools.to_msg(T, 'tool0', 'flange',
                                              w.robot.get_clock().now().to_msg()))
            time.sleep(0.05)
    threading.Thread(target=wiggle, daemon=True).start()
    try:
        out = w.result(w.driver.send(execution='moveit'), timeout=60.0)
    finally:
        stop.set()
    assert out.result.error_code == R.TF_UNAVAILABLE, out.result.message
    assert 'moving' in out.result.message


def test_ctrl_c_cancels_the_motion_before_exiting(moveit_world, tmp_path):
    """SIGINT to the node process while a motion runs: the node cancels the goal
    (while its executor still runs) before it exits, so no motion is left
    running without its calibrator."""
    import signal
    import subprocess

    from ament_index_python.packages import get_package_prefix
    from ros_harness import Driver
    import yaml
    w = moveit_world()
    w.moveit.motion_time = 30.0
    params = {k: v for k, v in MOVEIT.items()}
    params.update({'output_dir': str(tmp_path), 'session.target_captures': 10,
                   'session.frames_per_capture': 2, 'capture.consensus_min_frames': 2,
                   'solver.min_captures': 8, 'timing.settle_window': 0.3,
                   'timing.clock_tolerance': 10.0})
    nested: dict = {}
    for k, v in params.items():
        node = nested
        *head, last = k.split('.')
        for part in head:
            node = node.setdefault(part, {})
        node[last] = v
    pfile = tmp_path / 'p.yaml'
    pfile.write_text(yaml.safe_dump({'/**': {'ros__parameters': nested}}))
    exe = os.path.join(get_package_prefix('handeye_autopilot'), 'lib', 'handeye_autopilot',
                       'calibrate_node')
    log_path = tmp_path / 'node.log'
    log = open(log_path, 'w')
    proc = subprocess.Popen([exe, '--ros-args', '-r', '__node:=handeye_sub',
                             '--params-file', str(pfile)], stdout=log, stderr=subprocess.STDOUT,
                            env=dict(os.environ, ROS_DOMAIN_ID=DOMAIN, PYTHONUNBUFFERED='1'),
                            start_new_session=True)
    try:
        _wait(lambda: 'ready: send a goal' in log_path.read_text(), 60.0, 'node not ready')
        d = Driver(w.client, ns='/handeye_sub')
        assert d.action.wait_for_server(timeout_sec=60.0)
        # DDS discovery is per direction: under load the server can see our goal
        # request before it has matched our response reader, and the goal reply
        # is then dropped ('failed to send response'). Let discovery settle; if
        # the first reply is still lost, stop that run and send again.
        h = None
        for _attempt in range(3):
            time.sleep(5.0)
            try:
                h = d.send(execution='moveit', timeout=20.0)
                break
            except TimeoutError:
                d.call(d.cancel, timeout=20.0)
                _wait(lambda: 'run ended' in log_path.read_text(), 60.0, 'run did not end')
        assert h is not None and h.accepted
        _wait(lambda: w.moveit.goals, 120.0, 'no motion goal')
        time.sleep(0.5)
        proc.send_signal(signal.SIGINT)
        assert proc.wait(timeout=30) == 0
        assert w.moveit.cancels, 'the motion goal was not cancelled before the node exited'
    except BaseException:
        print(log_path.read_text()[-4000:])      # the node's side, for a CI failure
        raise
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=10)
        log.close()


def _cli(args, log_path):
    import subprocess

    from ament_index_python.packages import get_package_prefix
    exe = os.path.join(get_package_prefix('handeye_autopilot'), 'lib', 'handeye_autopilot',
                       'calibrate')
    return subprocess.Popen([exe] + args, stdout=open(log_path, 'w'),
                            stderr=subprocess.STDOUT,
                            env=dict(os.environ, ROS_DOMAIN_ID=DOMAIN, PYTHONUNBUFFERED='1'),
                            start_new_session=True)


def _cli_until_answered(args, tmp_path, ok_codes, attempts=3):
    """Run the CLI; a run in which DDS dropped the very first goal reply (the CLI
    then reports 'no goal reply', exits 2 and cancels its goal by ID) is retried:
    discovery under full-suite load, not what the test checks."""
    for k in range(attempts):
        log = tmp_path / f'cli{k}.log'
        proc = _cli(args, log)
        yield proc, log
        if proc.poll() is None or proc.returncode in ok_codes or \
                'no goal reply' not in log.read_text():
            return
        time.sleep(3.0)


def test_cli_ctrl_c_cancels_the_goal(moveit_world, tmp_path):
    """`ros2 run handeye_autopilot calibrate`: Ctrl-C cancels the goal and waits
    for the stop (exit 130)."""
    import signal
    w = moveit_world()
    w.moveit.motion_time = 30.0
    time.sleep(2.0)
    for proc, log in _cli_until_answered(['--mode', 'eye_to_hand', '--execution', 'moveit',
                                          '--accept-timeout', '20'], tmp_path, (130,)):
        try:
            _wait(lambda p=proc, lg=log: 'goal accepted' in lg.read_text() or
                  p.poll() is not None, 60.0, 'no goal reply')
            if proc.poll() is not None:            # reply lost: retried
                continue
            _wait(lambda: w.moveit.goals, 120.0, 'no motion goal')
            proc.send_signal(signal.SIGINT)
            assert proc.wait(timeout=60) == 130, log.read_text()
        finally:
            if proc.poll() is None:
                proc.kill()
    assert proc.returncode == 130, log.read_text()
    assert w.moveit.cancels
    assert 'CANCELED' in log.read_text()


def test_cli_without_a_goal_reply_cancels_its_own_goal(moveit_world, tmp_path):
    """The goal reply does not arrive in time (here: a server that admits slower
    than the accept timeout): the CLI cancels ITS goal by ID through the action's
    cancel service, waits for its result, and exits 2."""
    w = moveit_world()
    cancelled = _dummy_server(w.client, '/dummy/calibrate', admit_s=3.0, accept=True)
    time.sleep(2.0)
    log = tmp_path / 'cli.log'
    proc = _cli(['--mode', 'eye_to_hand', '--node', '/dummy', '--accept-timeout', '1'], log)
    assert proc.wait(timeout=60) == 2, log.read_text()
    assert len(cancelled) == 1, log.read_text()
    assert 'stopped' in log.read_text()


def test_cli_rejected_goal_leaves_the_other_run_alone(moveit_world, tmp_path):
    """A second operator's goal is rejected (busy): the CLI exits 3 and the
    first operator's run is not cancelled."""
    w = moveit_world()
    w.moveit.motion_time = 20.0
    h = w.driver.send(execution='moveit')
    _wait(lambda: w.moveit.goals, 120.0, 'no motion goal')
    for proc, log in _cli_until_answered(['--mode', 'eye_to_hand'], tmp_path, (3,)):
        proc.wait(timeout=60)
        text = log.read_text()
    assert proc.returncode == 3, text
    assert not w.moveit.cancels and w.node._run is not None
    w.driver.wait(h.cancel_goal_async(), 10.0)


def test_finish_is_refused_in_moveit_mode(moveit_world):
    """~/finish belongs to manual mode: in MoveIt mode the plan decides when to
    solve, so the request is refused rather than acknowledged and ignored."""
    w = moveit_world()
    h = w.driver.send(execution='moveit')
    # 'capturing' comes after bootstrap, centring and planning (minutes on a slow
    # runner): the refusal must hold in that phase, where manual mode accepts it
    _wait(lambda: any(f.phase == FB.PHASE_CAPTURING for f in w.driver.feedback), 300.0,
          'no capture')
    res = w.driver.call(w.driver.finish)
    assert not res.success
    w.driver.wait(h.cancel_goal_async(), 10.0)


def _dummy_server(node, name, admit_s, accept, first_instant=False):
    """An action server whose admission takes ``admit_s`` and then accepts or
    rejects (``first_instant``: the first goal is admitted at once); accepted
    goals run until cancelled (at most 60 s). Returns the cancelled ids."""
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    class Ids(list):
        admitting = False
    cancelled = Ids()
    state = {'first': first_instant}

    def admit(_goal):
        if state.pop('first', False):           # the first operator: admitted at once
            return GoalResponse.ACCEPT
        cancelled.admitting = True               # type: ignore[attr-defined]
        time.sleep(admit_s)
        return GoalResponse.ACCEPT if accept else GoalResponse.REJECT

    def execute(handle):
        t0 = time.monotonic()
        while not handle.is_cancel_requested and time.monotonic() - t0 < 60.0:
            time.sleep(0.05)
        r = Calibrate.Result(error_code=Calibrate.Result.CANCELED, message='cancelled')
        if handle.is_cancel_requested:
            handle.canceled(r)
        else:
            handle.abort(r)
        return r

    def on_cancel(handle):
        cancelled.append(bytes(handle.goal_id.uuid))
        return CancelResponse.ACCEPT
    ActionServer(node, Calibrate, name, execute, goal_callback=admit,
                 cancel_callback=on_cancel, callback_group=ReentrantCallbackGroup())
    return cancelled


def test_cli_interrupt_while_rejection_pending_spares_the_other_run(moveit_world, tmp_path):
    """Operator A's goal runs; operator B's CLI is interrupted while B's (busy)
    rejection is still pending: B cancels only B's goal ID, never A's run."""
    import signal

    from rclpy.action import ActionClient
    w = moveit_world()
    cancelled = _dummy_server(w.client, '/dummy2/calibrate', admit_s=4.0, accept=False,
                              first_instant=True)
    a = ActionClient(w.client, Calibrate, '/dummy2/calibrate')
    assert a.wait_for_server(timeout_sec=10.0)
    time.sleep(1.0)
    ha = w.driver.wait(a.send_goal_async(Calibrate.Goal(mode='eye_to_hand')), 10.0)
    assert ha.accepted
    log = tmp_path / 'cli.log'
    proc = _cli(['--mode', 'eye_to_hand', '--node', '/dummy2', '--accept-timeout', '30'], log)
    _wait(lambda: cancelled.admitting, 30.0, "B's goal never reached admission")
    time.sleep(0.5)
    proc.send_signal(signal.SIGINT)
    assert proc.wait(timeout=60) == 130, log.read_text()
    assert 'interrupted:' in log.read_text()     # the fallback cancel by ID ran
    assert bytes(ha.goal_id.uuid) not in cancelled, "operator A's run was cancelled"
    assert not ha.get_result_async().done()  # A's run still going
    w.driver.wait(ha.cancel_goal_async(), 10.0)


def test_cli_fallback_cancel_follows_a_remapped_action(moveit_world, tmp_path):
    """With the action name remapped, the fallback cancel (no goal reply) still
    reaches the server: it goes through the action client's own channels."""
    w = moveit_world()
    cancelled = _dummy_server(w.client, '/remapped/calibrate', admit_s=3.0, accept=True)
    time.sleep(2.0)
    log = tmp_path / 'cli.log'
    proc = _cli(['--mode', 'eye_to_hand', '--node', '/dummy3', '--accept-timeout', '1',
                 '--ros-args', '-r', '/dummy3/calibrate:=/remapped/calibrate'], log)
    try:
        assert proc.wait(timeout=60) == 2, log.read_text()
    finally:
        if proc.poll() is None:
            proc.kill()
    assert len(cancelled) == 1, log.read_text()        # the CLI's goal, at the remap
    assert 'stopped' in log.read_text()
