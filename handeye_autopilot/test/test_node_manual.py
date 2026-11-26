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
        self.driver.debug = self._debug
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

    def _debug(self):
        run = self.node._run
        im = self.node.images.latest()
        return {'run': None if run is None else (run.phase, run.capturing,
                                                 run.capture_request.is_set(), run.accepted,
                                                 run.rejected, run.last_detail[:80]),
                'node_now': round(self.node.now(), 3),
                'last_result': None if self.node.last_result is None else (
                    self.node.last_result.error_code, self.node.last_result.message[:300]),
                'latest_image': None if im is None else (round(im.stamp, 3),
                                                         round(im.received, 3)),
                'threads': [t.name for t in threading.enumerate()][:12]}

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
    w = world_factory(cam_over={'stamp_from_clock': True})
    w.robot.delay = 0.25                       # TF arrives 0.25 s late (< tf_timeout)
    time.sleep(0.5)
    assert w.driver.send().accepted
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, 3)
    assert w.driver.feedback[-1].accepted == 3


def test_tf_that_never_arrives_rejects_the_capture(world_factory):
    w = world_factory(cam_over={'stamp_from_clock': True})
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
    out = w.result(w.driver.send())
    res = out.result
    assert out.status == 6                               # GoalStatus ABORTED
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
    out = w.result(h)
    assert out.status == 6
    res = out.result
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


# -- review fixes (M3 xhigh pass 1) -------------------------------------------------

def test_each_frame_is_paired_with_the_pose_at_its_own_stamp(world_factory):
    """The robot keeps moving in small steps, with a settle loose enough to pass:
    every recorded tool pose must be exactly the pose published at the image's
    stamp, never the latest one."""
    w = world_factory(node_over={'timing.settle_translation': 1.0,
                                 'timing.settle_rotation': 3.0,
                                 'capture.consensus_max_translation_frac': 1.0,
                                 'capture.consensus_max_rotation': 3.0})
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    stop = threading.Event()
    base = w.poses[0]

    def creep():
        k = 0
        while not stop.is_set():
            k += 1
            w.robot.move_to(base @ tf.make(np.eye(3), [0.0005 * (k % 40), 0, 0]))
            time.sleep(0.02)
    t = threading.Thread(target=creep, daemon=True)
    t.start()
    d.capture_and_wait()
    stop.set()
    d.wait(h.cancel_goal_async())
    w.result(h)
    recs = [r for r in w.node.last_run_records if r.tool_pose is not None]
    assert recs, 'no record carried a tool pose'
    hist = dict(w.robot.history)
    for r in recs:
        ns = round(r.stamp * 1e9)
        match = min(hist, key=lambda s: abs(s - ns))
        assert abs(match - ns) < 2000                  # the very TF message of that stamp
        assert np.allclose(hist[match], r.tool_pose, atol=1e-9)


def test_clock_jump_during_a_capture_aborts(world_factory):
    w = world_factory(sim_time=True, node_over={'timing.settle_timeout': 30.0})
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    stop = threading.Event()

    def wobble():
        k = 0
        while not stop.is_set():
            k += 1
            w.robot.move_to(w.poses[k % 2])
            time.sleep(0.05)
    threading.Thread(target=wobble, daemon=True).start()
    assert d.call(d.capture).success
    d.wait_phase(FB.PHASE_CAPTURING)
    w.clock.jump(-100.0)
    out = w.result(h)
    res = out.result
    stop.set()
    assert out.status == 6                               # ABORTED, never SUCCEEDED
    assert res.error_code in (R.TF_UNAVAILABLE,), res.message
    assert 'jumped back' in res.message or 'clock' in res.message


def test_camera_switch_mid_run_aborts(world_factory):
    w = world_factory()
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    w.camera.optical = 'other_camera_optical_frame'    # images now from another frame
    time.sleep(0.3)
    d.call(d.capture)
    res = w.result(h).result
    assert res.error_code == R.NO_CAMERA_INFO


def test_capture_refused_once_the_run_is_finishing(world_factory, tmp_path):
    (tmp_path / 'tiny.yaml').write_text('session:\n  target_captures: 3\nsolver:\n'
                                        '  min_captures: 3\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path),
                                 'timing.settle_window': 0.3})
    d = w.driver
    h = d.send(profile='tiny')
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=3)
    r = d.call(d.capture)
    assert not r.success
    assert w.result(h).result.quality.captures_accepted == 3


def test_profile_with_a_string_boolean_is_invalid(world_factory, tmp_path):
    (tmp_path / 'strbool.yaml').write_text('image_is_rectified: "false"\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='strbool')).result
    assert res.error_code == R.INVALID_GOAL and 'image_is_rectified' in res.message


def test_replayed_joint_state_does_not_clear_the_fault(world_factory):
    from sensor_msgs.msg import JointState
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)

    def replay():
        time.sleep(0.3)
        m = JointState(name=['j1'], position=[0.1])
        m.header.stamp.sec = 10                       # an old, replayed message
        pub.publish(m)
    threading.Thread(target=replay, daemon=True).start()
    r = d.call(d.clear_fault)
    assert not r.success


def test_moving_camera_link_is_not_publishable(world_factory):
    """camera_link -> optical also appears on /tf (a moving link): refused."""
    from handeye_autopilot.ros import tf_tools
    from tf2_msgs.msg import TFMessage
    import yaml
    w = world_factory()
    pub = w.client.create_publisher(TFMessage, '/tf', 10)
    stop = threading.Event()

    def spam():
        while not stop.is_set():
            m = tf_tools.to_msg(w.camera.T_root_optical, 'camera_link',
                                'camera_optical_frame', w.client.get_clock().now().to_msg())
            pub.publish(TFMessage(transforms=[m]))
            time.sleep(0.1)
    threading.Thread(target=spam, daemon=True).start()
    time.sleep(0.5)
    res = _finish_run(w)
    stop.set()
    pub_doc = yaml.safe_load(open(res.result_file))['publish']
    assert not pub_doc['ok'] and '/tf' in pub_doc['reason']


def test_publish_result_rechecks_the_live_tree():
    from handeye_autopilot.ros.publish_result import check
    assert check({}, 'base_link', 'camera_link') == ''
    assert 'already has a parent' in check({'camera_link': 'mount'}, 'base_link',
                                           'camera_link')
    assert 'cycle' in check({'base_link': 'camera_link'}, 'base_link', 'camera_link')


# -- review fixes (M3 xhigh pass 2) -------------------------------------------------

def test_bad_streams_do_not_crash_the_node(world_factory):
    from sensor_msgs.msg import Image, JointState
    w = world_factory()
    m = Image(encoding='32FC1', width=2, height=2, step=8, data=bytes(16))
    w.node._on_image(m)                              # logged and dropped
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    stop = threading.Event()

    def empty():
        while not stop.is_set():
            j = JointState()
            j.header.stamp = w.client.get_clock().now().to_msg()
            pub.publish(j)
            time.sleep(0.02)
    threading.Thread(target=empty, daemon=True).start()
    r = d.call(d.clear_fault)
    stop.set()
    assert not r.success and 'no positions' in r.message


def test_one_stamp_repeated_is_not_fresh_evidence(world_factory):
    from sensor_msgs.msg import JointState
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    stamp = None
    stop = threading.Event()

    def replay():
        nonlocal stamp
        time.sleep(0.1)
        stamp = w.client.get_clock().now().to_msg()
        while not stop.is_set():
            j = JointState(name=['j1'], position=[0.1], velocity=[0.0])
            j.header.stamp = stamp
            pub.publish(j)
            time.sleep(0.05)
    threading.Thread(target=replay, daemon=True).start()
    r = d.call(d.clear_fault)
    stop.set()
    assert not r.success


def test_string_boolean_anywhere_in_a_profile_is_invalid(world_factory, tmp_path):
    (tmp_path / 'mask.yaml').write_text('capture:\n  masking: "false"\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='mask')).result
    assert res.error_code == R.INVALID_GOAL and 'capture.masking' in res.message


def test_profile_is_read_at_admission(world_factory, tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('session:\n  target_captures: 3\nsolver:\n  min_captures: 3\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    h = w.driver.send(profile='p')
    f.write_text('session:\n  step_safety: 0.0\n')       # edited after acceptance
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)          # the accepted text still applies
    w.driver.wait(h.cancel_goal_async())
    assert w.result(h).result.error_code == R.CANCELED


def test_capture_acknowledged_before_finish_is_taken(world_factory):
    w = world_factory()
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=8)
    n = len(w.node._run.records)
    w.robot.move_to(w.poses[-1])
    assert d.call(d.capture).success
    assert d.call(d.finish).success
    res = w.result(h).result
    assert res.error_code == R.NONE
    assert len(w.node.last_run_records) == n + 1


def test_brief_camera_info_change_aborts(world_factory):
    w = world_factory()
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    info = w.camera.info(w.client.get_clock().now().to_msg())
    info.k = list(info.k)
    info.k[0] += 50.0                                      # one CameraInfo with other fx
    w.node._on_info(info)
    res = w.result(h).result
    assert res.error_code == R.NO_CAMERA_INFO and 'changed' in res.message


def test_cancel_while_writing_the_result_ends_canceled(world_factory, monkeypatch):
    w = world_factory()
    d = w.driver
    real = w.node._write_results

    def slow(*a, **k):
        time.sleep(1.0)
        return real(*a, **k)
    monkeypatch.setattr(w.node, '_write_results', slow)
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=10)
    d.wait_phase(FB.PHASE_VERIFYING)
    d.wait(h.cancel_goal_async())
    res = w.result(h)
    assert res.status == 5 and res.result.error_code == R.CANCELED   # never SUCCEEDED


def test_clock_jump_between_goals_is_survived(world_factory):
    w = world_factory(sim_time=True)
    w.clock.jump(-100.0)
    time.sleep(1.0)
    assert w.node._jumps >= 1
    h = w.driver.send()                                   # a fresh goal in the new epoch
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    w.driver.capture_and_wait()
    assert w.driver.feedback[-1].accepted == 1
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_publish_result_refuses_a_changed_camera_subtree(tmp_path):
    from handeye_autopilot.core.results import transform_dict
    from handeye_autopilot.ros import tf_tools
    from handeye_autopilot.ros.publish_result import ResultPublisher
    import yaml
    T = tf.make(np.eye(3), [0.0, 0.0, 0.1])
    doc = {'publish': {'ok': True, 'transform': transform_dict(np.eye(4), 'base_link', 'cam'),
                       'subtree': transform_dict(T, 'cam', 'cam_optical')}}
    f = tmp_path / 'result.yaml'
    f.write_text(yaml.safe_dump(doc))
    node = ResultPublisher(str(f), listen=0.0)
    try:
        node.buffer.set_transform_static(tf_tools.to_msg(T, 'cam', 'cam_optical'), 'test')
        assert node.check_subtree() == ''
        moved = tf.make(np.eye(3), [0.25, 0.0, 0.1])          # driver restarted: 25 cm off
        node.buffer.set_transform_static(tf_tools.to_msg(moved, 'cam', 'cam_optical'), 'test')
        assert 'changed' in node.check_subtree()
    finally:
        node.destroy_node()


# -- review fixes (M3 xhigh pass 3) -------------------------------------------------

def test_cancel_is_honoured_even_if_writing_fails(world_factory, monkeypatch):
    w = world_factory()
    d = w.driver

    def broken(*a, **k):
        time.sleep(1.0)
        raise OSError('disk full')
    monkeypatch.setattr(w.node, '_write_results', broken)
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=10)
    d.wait_phase(FB.PHASE_VERIFYING)
    d.wait(h.cancel_goal_async())
    out = w.result(h)
    assert out.status == 5 and out.result.error_code == R.CANCELED


def test_clock_reset_while_writing_is_not_a_success(world_factory, monkeypatch):
    w = world_factory(sim_time=True)
    d = w.driver
    real = w.node._write_results

    def jump_then_write(*a, **k):
        w.clock.jump(-0.05)                       # a 50 ms rollback after the last check
        time.sleep(0.5)
        return real(*a, **k)
    monkeypatch.setattr(w.node, '_write_results', jump_then_write)
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=10)
    out = w.result(h)
    assert out.status == 6 and out.result.error_code == R.TF_UNAVAILABLE


def test_unreadable_profile_is_rejected_not_a_crash(world_factory, tmp_path):
    import os
    f = tmp_path / 'locked.yaml'
    f.write_text('session:\n  target_captures: 3\n')
    os.chmod(f, 0)
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    try:
        if os.access(f, os.R_OK):
            pytest.skip('running as root: permissions do not apply')
        assert not w.driver.send(profile='locked').accepted
        assert w.driver.send().accepted                   # the node is still alive
    finally:
        os.chmod(f, 0o644)


@pytest.mark.parametrize('text,key', [('board:\n  squares_x: 7.5\n', 'board.squares_x'),
                                      ('timing:\n  tf_timeout: 5.0\n', 'tf_timeout')])
def test_more_profile_types_are_checked(world_factory, tmp_path, text, key):
    (tmp_path / 'bad.yaml').write_text(text)
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='bad')).result
    assert res.error_code == R.INVALID_GOAL and key in res.message


def test_burst_then_replay_does_not_clear_the_fault(world_factory):
    from sensor_msgs.msg import JointState
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    stop = threading.Event()

    def burst():
        time.sleep(0.1)
        last = None
        for _ in range(5):                         # five stamps within ~40 ms
            last = w.client.get_clock().now().to_msg()
            pub.publish(JointState(header=JointState().header.__class__(stamp=last),
                                   name=['j1'], position=[0.1], velocity=[0.0]))
            time.sleep(0.008)
        while not stop.is_set():                   # then replay the last one
            pub.publish(JointState(header=JointState().header.__class__(stamp=last),
                                   name=['j1'], position=[0.1], velocity=[0.0]))
            time.sleep(0.05)
    threading.Thread(target=burst, daemon=True).start()
    r = d.call(d.clear_fault)
    stop.set()
    assert not r.success


def test_publish_result_refuses_a_moving_subtree_link(tmp_path):
    from handeye_autopilot.core.results import transform_dict
    from handeye_autopilot.ros import tf_tools
    from handeye_autopilot.ros.publish_result import ResultPublisher
    from tf2_msgs.msg import TFMessage
    import yaml
    T = tf.make(np.eye(3), [0.0, 0.0, 0.1])
    doc = {'publish': {'ok': True, 'transform': transform_dict(np.eye(4), 'base_link', 'cam'),
                       'subtree': transform_dict(T, 'cam', 'cam_optical')}}
    f = tmp_path / 'result.yaml'
    f.write_text(yaml.safe_dump(doc))
    node = ResultPublisher(str(f), listen=0.0)
    try:
        m = tf_tools.to_msg(T, 'cam', 'cam_optical')
        node.buffer.set_transform(m, 'test')               # same pose, but on /tf
        node.moving.add(TFMessage(transforms=[m]))
        assert 'moving' in node.check_subtree()
    finally:
        node.destroy_node()
