"""Real MoveIt (Panda on mock hardware), 03-milestones M4 failure tests:

- the ``compute_ik`` contract (timeout behaviour) on this distro;
- collision rejection by ``compute_ik`` (the hand inside the table);
- a pose where ONLY the attached board collides is rejected, and accepted once
  the board is detached: attached bodies take part in the IK collision check
  (requests use the monitored scene state, ``is_diff``, 01-architecture §3.1);
- the demo end to end (``demo.launch.py``, eye-to-hand, unattended): it must
  print DEMO PASSED.

Each test starts its own launch in its own ROS domain, so the in-process fake
MoveIt tests of handeye_autopilot (run in parallel by colcon) never see it.
Skipped when the Panda config is not installed (HANDEYE_AUTOPILOT_REQUIRE_DEPS=1
in CI turns the skip into a failure).
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')


def _require(pkg):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        get_package_share_directory(pkg)
    except PackageNotFoundError:
        if os.environ.get('HANDEYE_AUTOPILOT_REQUIRE_DEPS') == '1':
            raise
        pytest.skip(f'{pkg} not installed')


DOMAIN = '58'


def _launch(args, log, domain=DOMAIN):
    env = dict(os.environ, ROS_DOMAIN_ID=domain)
    return subprocess.Popen(['ros2', 'launch', 'handeye_autopilot_demos'] + args,
                            stdout=log, stderr=subprocess.STDOUT, env=env,
                            start_new_session=True)          # own process group


def _stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=10)


@pytest.fixture(scope='module')
def panda(tmp_path_factory):
    _require('moveit_resources_panda_moveit_config')
    log = open(tmp_path_factory.mktemp('panda') / 'launch.log', 'w')
    proc = _launch(['panda_robot.launch.py'], log)
    scene = subprocess.Popen(['ros2', 'run', 'handeye_autopilot_demos', 'scene', '--ros-args',
                              '-p', 'robot:=panda', '-p', 'mode:=eye_to_hand'],
                             stdout=log, stderr=subprocess.STDOUT,
                             env=dict(os.environ, ROS_DOMAIN_ID=DOMAIN),
                             start_new_session=True)
    old = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = DOMAIN
    rclpy.init()
    from handeye_autopilot.ros.moveit_client import MoveItClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    node = rclpy.create_node('panda_contract_test')
    client = MoveItClient(node, ReentrantCallbackGroup())
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        assert not client.missing(120.0), 'move_group did not come up'
        ready = _scene_ready(node)
        assert ready.wait(120.0), 'the demo scene was not applied'
        time.sleep(1.0)
        yield node, client
    finally:
        ex.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
        if old is None:
            del os.environ['ROS_DOMAIN_ID']
        else:
            os.environ['ROS_DOMAIN_ID'] = old
        _stop(scene)
        _stop(proc)
        log.close()


def _scene_ready(node):
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool
    ev = threading.Event()
    node.create_subscription(Bool, '/demo/scene_ready', lambda m: m.data and ev.set(),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    return ev


JOINTS = [f'panda_joint{i}' for i in range(1, 8)]


def _hand_down(x, y, z):
    """panda_link8 pose with the hand pointing down (as at the ready pose)."""
    from handeye_autopilot.core import transforms as tf
    import numpy as np
    from scipy.spatial.transform import Rotation
    return tf.make(Rotation.from_euler('xyz', (math.pi, 0, -math.pi / 4)), np.array([x, y, z]))


def _ik(client, G, timeout=1):
    return client.solve_ik(G, 'panda_link0', 'panda_arm', 'panda_link8', JOINTS, timeout)


def test_ik_contract_whole_seconds(panda):
    """§3.1: one request per candidate; a whole-second timeout is honoured (an
    unreachable pose uses all of it), and a reachable pose solves."""
    node, client = panda
    assert _ik(client, _hand_down(0.4, 0.0, 0.5)).joints is not None
    t0 = time.monotonic()
    res = _ik(client, _hand_down(3.0, 0.0, 0.5), timeout=1)
    took = time.monotonic() - t0
    assert res.joints is None and res.reason == 'NO_IK_SOLUTION'
    assert 0.8 < took < 5.0, took
    # recorded, not asserted: whether this distro honours nanoseconds (Jazzy drops
    # them, probe finding 6); the node only ever sends whole seconds
    print(f'compute_ik unreachable with timeout 1 s took {took:.2f} s')


def test_compute_ik_rejects_a_pose_in_collision(panda):
    node, client = panda
    assert _ik(client, _hand_down(0.45, 0.25, 0.45)).joints is not None
    res = _ik(client, _hand_down(0.45, 0.25, 0.02))      # hand inside the table
    assert res.joints is None and res.reason == 'NO_IK_SOLUTION'


def test_only_the_attached_board_collides(panda):
    """The board hangs ~0.2 m below the hand: at this height the hand and fingers
    clear the table but the board does not. Rejected with the board attached,
    accepted without it: the attached body is in the IK collision check."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    node, client = panda
    G = _hand_down(0.45, 0.0, 0.17)
    with_board = _ik(client, G)
    apply = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    assert apply.wait_for_service(timeout_sec=10.0)

    def scene_call(scene):
        fut = apply.call_async(ApplyPlanningScene.Request(scene=scene))
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 10:
            time.sleep(0.02)
        assert fut.done() and fut.result().success
    detach = PlanningScene(is_diff=True)
    detach.robot_state.is_diff = True
    gone = AttachedCollisionObject(link_name='panda_hand')
    gone.object.id = 'calibration_board'
    gone.object.operation = CollisionObject.REMOVE
    detach.robot_state.attached_collision_objects = [gone]
    scene_call(detach)
    # detaching leaves the object in the world: remove it there too
    world = PlanningScene(is_diff=True)
    world.world.collision_objects = [CollisionObject(id='calibration_board',
                                                     operation=CollisionObject.REMOVE)]
    scene_call(world)
    time.sleep(0.5)
    without_board = _ik(client, G)
    assert with_board.joints is None and with_board.reason == 'NO_IK_SOLUTION'
    assert without_board.joints is not None, without_board.reason


def test_panda_demo_eye_to_hand_end_to_end(tmp_path):
    """03-milestones M4 done-when: the demo runs unattended and passes."""
    _require('moveit_resources_panda_moveit_config')
    log_path = tmp_path / 'demo.log'
    with open(log_path, 'w') as log:
        # its own domain: the module fixture's move_group may still be running
        proc = _launch(['demo.launch.py', 'robot:=panda', 'mode:=eye_to_hand'], log, '59')
        try:
            proc.wait(timeout=float(os.environ.get('HANDEYE_DEMO_TIMEOUT', '1200')))
        except subprocess.TimeoutExpired:
            pass
        finally:
            _stop(proc)
    text = log_path.read_text()
    lines = [ln for ln in text.splitlines() if 'demo_runner' in ln]
    assert 'DEMO PASSED' in text, '\n'.join(lines[-30:]) or text[-3000:]
