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
    os.environ['ROS_DOMAIN_ID'] = '57'
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

    def plan_fails(joints):               # the first two planned moves cannot be planned:
        if fails['n'] < 2:                # safe skips, spares take their place
            fails['n'] += 1
            return True
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


@pytest.mark.parametrize('fault', ['jump', 'nan_fraction'])
def test_bad_cartesian_paths_are_never_executed(moveit_world, fault):
    """A path with a joint jump (some releases ignore the threshold field) or a
    NaN fraction is refused before anything is sent: not_started, nothing runs."""
    w = moveit_world()
    if fault == 'jump':
        w.moveit.cart_jump = True
    else:
        w.moveit.cart_fraction = float('nan')
    out = w.result(w.driver.send(execution='moveit'), timeout=120.0)
    assert out.result.error_code == R.NOT_ENOUGH_SAMPLES, out.result.message
    assert 'bootstrap_failed' in out.result.message
    assert not [k for k, _ in w.moveit.goals if k == 'execute']


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
            T = tf.make(np.eye(3), np.array([0.0, 0.0, 0.1 + 0.01 * (k % 7)]))
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
