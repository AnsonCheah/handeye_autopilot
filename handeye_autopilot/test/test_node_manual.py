"""M3 node tests, in one process (03-milestones M3 done-when): a full manual
calibration with the synthetic camera and scripted captures in both modes, the
failure paths (delayed TF, mismatched stamps, frozen joint states, clock jump,
TF ownership conflict, cancel during capture) and admission rejections.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')

from handeye_autopilot.core import transforms as tf  # noqa: E402
from handeye_autopilot.ros.node import CalibrationNode  # noqa: E402
from handeye_autopilot.ros.synthetic_camera import SyntheticCamera  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from ros_harness import Driver, FakeRobot, SimClock, camera_params, scripted_poses  # noqa: E402,E501

from handeye_autopilot_interfaces.action import Calibrate  # noqa: E402

R = Calibrate.Result
FB = Calibrate.Feedback


def params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


class World:
    """Node + synthetic camera + fake robot + client, spinning in one executor."""

    def __init__(self, mode, tmp_path, node_over=None, cam_over=None, sim_time=False):
        sim = {'use_sim_time': True} if sim_time else {}
        self.clock = SimClock() if sim_time else None
        self.node = CalibrationNode(parameter_overrides=params({**sim,
            'output_dir': str(tmp_path), 'session.target_captures': 10,
            'session.frames_per_capture': 2, 'capture.consensus_min_frames': 2,
            'solver.min_captures': 8, 'testing.allow_fault_injection': True,
            'timing.settle_window': 0.3, 'timing.settle_timeout': 3.0,
            **(node_over or {})}))
        self.camera = SyntheticCamera(parameter_overrides=params(
            dict(camera_params(mode, **(cam_over or {})), **sim)))
        self.robot = FakeRobot(parameter_overrides=params(sim))
        self.client = Node('test_client')
        self.driver = Driver(self.client)
        self.ex = MultiThreadedExecutor(num_threads=8)
        self.nodes = [self.node, self.camera, self.robot, self.client] + \
            ([self.clock] if self.clock else [])
        for n in self.nodes:
            self.ex.add_node(n)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        self.mode = mode
        self.X, self.Z = self.camera.true_X, self.camera.Z
        self.poses = scripted_poses(mode, self.X, self.Z, self.camera.camera,
                                    self.camera.board, n=14)
        self.robot.move_to(self.poses[0])
        time.sleep(1.0)            # TF, camera_info and images flowing

    def close(self):
        run = self.node._run                  # never leave a run blocking shutdown
        if run is not None:
            run.cancelled.set()
            t0 = time.monotonic()
            while self.node._run is not None and time.monotonic() - t0 < 5:
                time.sleep(0.05)
        self.ex.shutdown(timeout_sec=2.0)
        for n in self.nodes:
            n.destroy_node()

    def result(self, handle, timeout=120.0):
        return self.driver.wait(handle.get_result_async(), timeout)


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def world_factory(tmp_path):
    worlds = []

    def make(mode='eye_to_hand', **kw):
        w = World(mode, tmp_path, **kw)
        worlds.append(w)
        return w
    yield make
    for w in worlds:
        w.close()


@pytest.mark.parametrize('mode', ['eye_to_hand', 'eye_in_hand'])
def test_manual_calibration_recovers_the_true_mount(world_factory, mode):
    w = world_factory(mode)
    d = w.driver
    h = d.send(mode=mode)
    assert h.accepted
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    for G in w.poses:
        if d.feedback[-1].accepted >= 10:
            break
        w.robot.move_to(G)
        d.capture_and_wait()
    res = w.result(h).result
    assert res.error_code == R.NONE, res.message
    assert res.transform_valid
    X = tf.make(np.eye(3), np.zeros(3))
    t, q = res.camera_transform.transform.translation, res.camera_transform.transform.rotation
    from scipy.spatial.transform import Rotation
    X[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    X[:3, 3] = [t.x, t.y, t.z]
    assert tf.translation_distance(X, w.X) < 0.003
    assert math.degrees(tf.rotation_angle(X, w.X)) < 0.5
    parent = 'base_link' if mode == 'eye_to_hand' else 'tool0'
    assert res.camera_transform.header.frame_id == parent
    assert res.camera_transform.child_frame_id == 'camera_optical_frame'
    assert res.result_file and res.quality.captures_accepted >= 8


def run_captures(w, n, accepted_target=None):
    d = w.driver
    for G in w.poses[:n]:
        w.robot.move_to(G)
        d.capture_and_wait()
        if accepted_target and d.feedback[-1].accepted >= accepted_target:
            break


# -- timing and clock (§8) -----------------------------------------------------

def test_delayed_tf_within_the_timeout_is_waited_for(world_factory):
    w = world_factory()
    w.robot.delay = 0.25                       # TF arrives 0.25 s late (< tf_timeout)
    time.sleep(0.5)
    assert w.driver.send().accepted
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, 3)
    assert w.driver.feedback[-1].accepted == 3


def test_tf_that_never_arrives_rejects_the_capture(world_factory):
    w = world_factory()
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    w.robot.delay = 2.0                        # later than tf_timeout: never substituted
    time.sleep(0.1)
    w.driver.capture_and_wait()
    fb = w.driver.feedback[-1]
    assert fb.rejected == 1 and fb.accepted == 0
    assert fb.last_rejection in (FB.REJECT_TF_UNAVAILABLE,)
    h.cancel_goal_async()


def test_mismatched_image_stamps_abort_at_preflight(world_factory):
    w = world_factory(cam_over={'stamp_offset': 5.0})
    res = w.result(w.driver.send()).result
    assert res.error_code == R.TF_UNAVAILABLE and 'clock' in res.message
    assert not res.transform_valid


def test_frozen_joint_states_abort_the_run(world_factory):
    w = world_factory()
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    w.robot.freeze = True
    time.sleep(0.2)
    w.driver.call(w.driver.capture)
    res = w.result(h).result
    assert res.error_code == R.TF_UNAVAILABLE and 'frozen' in res.message


def test_clock_jump_aborts_the_run(world_factory):
    w = world_factory(sim_time=True)
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    w.clock.jump(-100.0)                       # sim time reset
    res = w.result(h).result
    assert res.error_code == R.TF_UNAVAILABLE and 'jumped back' in res.message


# -- cancel ------------------------------------------------------------------------

def test_cancel_during_capture(world_factory):
    w = world_factory(node_over={'timing.settle_timeout': 30.0})
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    stop = threading.Event()

    def wobble():                              # the robot never settles
        k = 0
        while not stop.is_set():
            k += 1
            w.robot.move_to(w.poses[k % 2])
            time.sleep(0.05)
    t = threading.Thread(target=wobble, daemon=True)
    t.start()
    assert w.driver.call(w.driver.capture).success
    w.driver.wait_phase(FB.PHASE_CAPTURING)
    t0 = time.monotonic()
    w.driver.wait(h.cancel_goal_async())
    res = w.result(h)
    stop.set()
    assert res.status == 5 and res.result.error_code == R.CANCELED     # GoalStatus CANCELED
    assert time.monotonic() - t0 < 5.0


# -- results and TF ownership (§9) ----------------------------------------------------

def _finish_run(w, mode='eye_to_hand'):
    h = w.driver.send(mode=mode)
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=10)
    return w.result(h).result


def test_result_yaml_publishes_parent_to_camera_root(world_factory):
    import yaml
    w = world_factory()
    res = _finish_run(w)
    assert res.error_code == R.NONE, res.message
    doc = yaml.safe_load(open(res.result_file))
    pub = doc['publish']
    assert pub['ok'], pub['reason']
    t = pub['transform']
    assert t['parent'] == 'base_link' and t['child'] == 'camera_link'
    true_root = w.camera.T_parent_root
    assert np.linalg.norm(np.array(t['translation']) - true_root[:3, 3]) < 0.003
    obs = res.result_file.replace('result.yaml', 'observations.yaml')
    from handeye_autopilot.core import observations
    recs, _, meta = observations.read(obs)
    assert meta['mode'] == 'eye_to_hand' and sum(r.accepted for r in recs) >= 10


@pytest.mark.parametrize('case', ['root_has_parent', 'cycle'])
def test_tf_ownership_conflict_is_refused_but_the_result_kept(world_factory, case):
    """(a) the driver root camera_link already has a parent (e.g. an old mounting
    joint); (b) the camera is already attached to the robot tree, so its topmost
    ancestor is the base itself (publishing would make a cycle)."""
    from handeye_autopilot.ros import tf_tools
    import tf2_ros
    import yaml
    over = {'camera_root_frame': 'camera_link'} if case == 'root_has_parent' else {}
    w = world_factory(node_over=over)
    b = tf2_ros.StaticTransformBroadcaster(w.client)
    parent = 'old_mount' if case == 'root_has_parent' else 'base_link'
    b.sendTransform(tf_tools.to_msg(tf.make(np.eye(3), [0, 0, 1.0]), parent, 'camera_link',
                                    w.client.get_clock().now().to_msg()))
    time.sleep(0.5)
    res = _finish_run(w)
    assert res.error_code == R.NONE and res.transform_valid      # calibration kept
    pub = yaml.safe_load(open(res.result_file))['publish']
    assert not pub['ok']
    if case == 'root_has_parent':
        assert 'already has a parent' in pub['reason'] and 'old_mount' in pub['reason']
    else:
        assert 'cycle' in pub['reason']


# -- admission (§5): rejected goals get no result -------------------------------------

def test_admission_rejections(world_factory, tmp_path):
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    d = w.driver
    assert not d.send(mode='sideways').accepted
    assert not d.send(execution='teleport').accepted
    assert not d.send(profile='nope').accepted
    assert not d.send(profile='../etc').accepted
    h = d.send()
    assert h.accepted
    assert not d.send().accepted                    # busy
    d.wait(h.cancel_goal_async())
    w.result(h)
    assert d.call(d.inject_fault).success
    assert not d.send().accepted                    # latched fault
    stop = threading.Event()

    def joints():                                    # a still robot for ~/clear_fault
        from sensor_msgs.msg import JointState
        pub = w.client.create_publisher(JointState, '/joint_states', 10)
        while not stop.is_set():
            m = JointState(name=['j1'], position=[0.1], velocity=[0.0])
            m.header.stamp = w.client.get_clock().now().to_msg()
            pub.publish(m)
            time.sleep(0.02)
    t = threading.Thread(target=joints, daemon=True)
    t.start()
    r = d.call(d.clear_fault)
    stop.set()
    assert r.success, r.message
    h = d.send()
    assert h.accepted
    d.wait(h.cancel_goal_async())
    w.result(h)


def test_clear_fault_needs_fresh_joint_states(world_factory):
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    r = d.call(d.clear_fault)
    assert not r.success and 'joint_states' in r.message
    assert not d.send().accepted


def test_profiles_override_and_validate(world_factory, tmp_path):
    (tmp_path / 'small.yaml').write_text('session:\n  target_captures: 3\n'
                                         'solver:\n  min_captures: 3\n')
    (tmp_path / 'broken.yaml').write_text('session:\n  step_safety: 0.0\n')
    (tmp_path / 'typo.yaml').write_text('sesion:\n  target_captures: 3\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    d = w.driver
    for bad in ('broken', 'typo'):
        res = w.result(d.send(profile=bad)).result
        assert res.error_code == R.INVALID_GOAL, res.message
    h = d.send(profile='small')
    assert h.accepted
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=3)
    res = w.result(h).result                       # solves after 3 (profile), not 10
    assert res.quality.captures_accepted == 3


def test_services_refuse_outside_their_phase(world_factory):
    w = world_factory()
    d = w.driver
    assert not d.call(d.capture).success              # no run
    assert not d.call(d.finish).success
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    r = d.call(d.finish)
    assert not r.success and 'at least 8' in r.message
    d.wait(h.cancel_goal_async())
    w.result(h)

