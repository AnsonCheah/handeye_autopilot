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
            # nine nodes share one Python process here: under full-suite load a
            # callback can stall for seconds, which a deployment (separate
            # processes) does not see; clock tests use offsets far beyond this
            'timing.clock_tolerance': 10.0,
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
    w = world_factory(cam_over={'stamp_offset': 60.0})
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
    if case == 'root_has_parent':
        msg = tf_tools.to_msg(tf.make(np.eye(3), [0, 0, 1.0]), 'old_mount', 'camera_link',
                              w.client.get_clock().now().to_msg())
    else:                                   # base_link hangs below the camera: a cycle
        msg = tf_tools.to_msg(tf.make(np.eye(3), [0, 0, 1.0]), 'camera_link', 'base_link',
                              w.client.get_clock().now().to_msg())
    b.sendTransform(msg)
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
    (tmp_path / 'small.yaml').write_text('session:\n  target_captures: 5\n'
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
    run_captures(w, len(w.poses), accepted_target=5)
    res = w.result(h).result                       # solves after 5 (profile), not 10
    assert res.message                             # the real result, not rclpy's default
    assert res.quality.captures_accepted == 5


def test_services_refuse_outside_their_phase(world_factory):
    w = world_factory()
    d = w.driver
    assert not d.call(d.capture).success              # no run
    assert not d.call(d.finish).success
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    r = d.call(d.finish)
    assert not r.success and 'at least 10' in r.message      # 8 per fold x 5 folds
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
    (tmp_path / 'tiny.yaml').write_text('session:\n  target_captures: 5\nsolver:\n'
                                        '  min_captures: 3\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path),
                                 'timing.settle_window': 0.3})
    d = w.driver
    h = d.send(profile='tiny')
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=5)
    r = d.call(d.capture)
    assert not r.success
    assert w.result(h).result.quality.captures_accepted == 5


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
    f.write_text('session:\n  target_captures: 5\nsolver:\n  min_captures: 3\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    h = w.driver.send(profile='p')
    f.write_text('session:\n  step_safety: 0.0\n')       # edited after acceptance
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE)          # the accepted text still applies
    w.driver.wait(h.cancel_goal_async())
    assert w.result(h).result.error_code == R.CANCELED


def test_capture_acknowledged_before_finish_is_taken(world_factory):
    w = world_factory(node_over={'session.target_captures': 12})
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=10)      # finish needs 10 (5 folds x 8)
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
    if res.result.result_file:                       # a file written first is unpublishable
        import yaml
        assert not yaml.safe_load(open(res.result.result_file))['publish']['ok']


def test_clock_jump_between_goals_is_survived(world_factory):
    w = world_factory(sim_time=True, cam_over={'stamp_from_clock': True})
    w.robot.delay = 0.15                          # TF in flight across the reset
    time.sleep(0.5)
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
    doc = {'passed': True, 'transform_valid': True,
           'publish': {'ok': True, 'transform': transform_dict(np.eye(4), 'base_link', 'cam'),
                       'subtree': transform_dict(T, 'cam', 'cam_optical')}}
    f = tmp_path / 'result.yaml'
    f.write_text(yaml.safe_dump(doc))
    (tmp_path / '.publishable').touch()               # as after a SUCCEEDED run
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
    doc = {'passed': True, 'transform_valid': True,
           'publish': {'ok': True, 'transform': transform_dict(np.eye(4), 'base_link', 'cam'),
                       'subtree': transform_dict(T, 'cam', 'cam_optical')}}
    f = tmp_path / 'result.yaml'
    f.write_text(yaml.safe_dump(doc))
    (tmp_path / '.publishable').touch()               # as after a SUCCEEDED run
    node = ResultPublisher(str(f), listen=0.0)
    try:
        # published for real on /tf by another node, received through the
        # publisher's own subscription (not by filling its tracker directly)
        import rclpy.node
        other = rclpy.node.Node('moving_tf_source')
        pub = other.create_publisher(TFMessage, '/tf', 10)
        t0 = time.monotonic()
        while not node.moving.has('cam', 'cam_optical') and time.monotonic() - t0 < 5:
            m = tf_tools.to_msg(T, 'cam', 'cam_optical', other.get_clock().now().to_msg())
            pub.publish(TFMessage(transforms=[m]))
            rclpy.spin_once(node, timeout_sec=0.05)
        assert 'moving' in node.check_subtree()
        other.destroy_node()
    finally:
        node.destroy_node()


# -- review fixes (M3 xhigh pass 4) -------------------------------------------------

def test_invalid_utf8_profile_is_rejected(world_factory, tmp_path):
    (tmp_path / 'bin.yaml').write_bytes(b'session:\n  target_captures: \xff\xfe\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    assert not w.driver.send(profile='bin').accepted
    assert w.driver.send().accepted                      # still alive


def test_frames_below_the_consensus_minimum_are_invalid(world_factory, tmp_path):
    (tmp_path / 'few.yaml').write_text('session:\n  frames_per_capture: 1\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='few')).result
    assert res.error_code == R.INVALID_GOAL and 'consensus_min_frames' in res.message


def test_nan_joint_states_do_not_clear_the_fault(world_factory):
    from sensor_msgs.msg import JointState
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    stop = threading.Event()

    def nans():
        while not stop.is_set():
            j = JointState(name=['j1'], position=[float('nan')], velocity=[float('nan')])
            j.header.stamp = w.client.get_clock().now().to_msg()
            pub.publish(j)
            time.sleep(0.02)
    threading.Thread(target=nans, daemon=True).start()
    r = d.call(d.clear_fault)
    stop.set()
    assert not r.success and 'NaN' in r.message


# -- review fixes (M3 xhigh pass 5) -------------------------------------------------

def test_result_without_the_success_marker_is_not_published(tmp_path):
    from handeye_autopilot.core.results import transform_dict
    from handeye_autopilot.ros.publish_result import ResultPublisher
    import yaml
    doc = {'passed': True, 'transform_valid': True,
           'publish': {'ok': True, 'transform': transform_dict(np.eye(4), 'base_link', 'cam')}}
    f = tmp_path / 'result.yaml'
    f.write_text(yaml.safe_dump(doc))
    with pytest.raises(SystemExit, match='publishable'):
        ResultPublisher(str(f), listen=0.0)


def test_successful_run_leaves_the_marker(world_factory):
    import os
    w = world_factory()
    res = _finish_run(w)
    assert res.error_code == R.NONE
    assert os.path.exists(os.path.join(os.path.dirname(res.result_file), '.publishable'))


def test_competing_parent_on_tf_is_refused():
    from handeye_autopilot.ros import tf_tools
    from tf2_msgs.msg import TFMessage
    static, moving = tf_tools.StaticLinks(), tf_tools.StaticLinks()
    static.add(TFMessage(transforms=[tf_tools.to_msg(np.eye(4), 'cam', 'opt')]))
    moving.add(TFMessage(transforms=[tf_tools.to_msg(np.eye(4), 'other_driver', 'opt')]))
    sub = tf_tools.camera_subtree({'opt': 'cam'}, static, 'opt', 'base_link', '', moving)
    assert sub.error and 'other_driver' in sub.error


def test_unexpandable_profiles_dir_is_a_rejection(world_factory):
    w = world_factory(node_over={'profiles_dir': '~no_such_user_xyz/profiles'})
    assert not w.driver.send(profile='p').accepted
    assert w.driver.send().accepted


def test_target_below_the_solver_minimum_is_invalid(world_factory, tmp_path):
    (tmp_path / 'low.yaml').write_text('session:\n  target_captures: 4\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='low')).result
    assert res.error_code == R.INVALID_GOAL and 'min_captures' in res.message


def test_a_moving_joint_on_another_publisher_blocks_clearing(world_factory):
    from sensor_msgs.msg import JointState
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    stop = threading.Event()

    def two_publishers():
        k = 0
        while not stop.is_set():
            k += 1
            now = w.client.get_clock().now().to_msg()
            pub.publish(JointState(header=JointState().header.__class__(stamp=now),
                                   name=['a1', 'a2'], position=[0.1, 0.2]))
            pub.publish(JointState(header=JointState().header.__class__(stamp=now),
                                   name=['rail'], position=[0.01 * k]))     # moving, no velocity
            time.sleep(0.02)
    threading.Thread(target=two_publishers, daemon=True).start()
    r = d.call(d.clear_fault)
    stop.set()
    assert not r.success and 'changing' in r.message


# -- review fixes (M3 xhigh pass 6) -------------------------------------------------

def test_conflicting_static_values_are_refused_at_publish_time(tmp_path):
    """Two live publishers disagree about a static subtree edge: publish_result
    (a fresh subscriber, so it sees only live publishers) refuses."""
    from handeye_autopilot.core.results import transform_dict
    from handeye_autopilot.ros import tf_tools
    from handeye_autopilot.ros.publish_result import ResultPublisher
    from tf2_msgs.msg import TFMessage
    import yaml
    T = tf.make(np.eye(3), [0.0, 0.0, 0.1])
    doc = {'passed': True, 'transform_valid': True,
           'publish': {'ok': True, 'transform': transform_dict(np.eye(4), 'base_link', 'cam'),
                       'subtree': transform_dict(T, 'cam', 'opt')}}
    f = tmp_path / 'result.yaml'
    f.write_text(yaml.safe_dump(doc))
    (tmp_path / '.publishable').touch()
    node = ResultPublisher(str(f), listen=0.0)
    try:
        node.buffer.set_transform_static(tf_tools.to_msg(T, 'cam', 'opt'), 'test')
        for M in (T, tf.make(np.eye(3), [0.25, 0, 0.1])):      # a URDF disagrees
            node.static_values.add(TFMessage(transforms=[tf_tools.to_msg(M, 'cam', 'opt')]))
        assert 'conflicting' in node.check_subtree()
    finally:
        node.destroy_node()


def test_preflight_uses_a_newer_frame_when_an_early_one_predates_tf(world_factory):
    """Camera streaming before the robot: the first frames predate any TF; the
    preflight must move on to newer frames instead of aborting."""
    w = world_factory(cam_over={'stamp_from_clock': True})
    w.robot.freeze = True
    w.node.tf_buffer.clear()
    time.sleep(0.5)                                # only TF-less images now
    h = w.driver.send()

    def start_robot():
        time.sleep(0.5)
        w.robot.freeze = False
    threading.Thread(target=start_robot, daemon=True).start()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=15.0)
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_target_must_leave_each_fold_enough_captures(world_factory, tmp_path):
    (tmp_path / 'eight.yaml').write_text('session:\n  target_captures: 8\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='eight')).result
    assert res.error_code == R.INVALID_GOAL and 'fold' in res.message


def test_one_sample_of_a_moving_joint_is_not_enough(world_factory):
    from sensor_msgs.msg import JointState
    w = world_factory()
    d = w.driver
    assert d.call(d.inject_fault).success
    pub = w.client.create_publisher(JointState, '/joint_states', 10)
    stop = threading.Event()

    def arm_and_one_rail_sample():
        first = True
        while not stop.is_set():
            now = w.client.get_clock().now().to_msg()
            pub.publish(JointState(header=JointState().header.__class__(stamp=now),
                                   name=['a1'], position=[0.1]))
            if first:
                pub.publish(JointState(header=JointState().header.__class__(stamp=now),
                                       name=['rail'], position=[0.5]))
                first = False
            time.sleep(0.02)
    threading.Thread(target=arm_and_one_rail_sample, daemon=True).start()
    r = d.call(d.clear_fault)
    stop.set()
    assert not r.success and 'rail' in r.message


# -- review fixes (M3 xhigh pass 7) -------------------------------------------------

def test_tf_trackers_stay_small_on_a_moving_robot():
    from handeye_autopilot.ros import tf_tools
    from tf2_msgs.msg import TFMessage
    moving, static = tf_tools.StaticLinks(), tf_tools.StaticLinks(track_values=True)
    t0 = time.monotonic()
    for k in range(20000):                        # ~5 min of one link at 60 Hz
        m = tf_tools.to_msg(tf.make(np.eye(3), [k * 1e-4, 0, 0]), 'base_link', 'tool0')
        moving.add(TFMessage(transforms=[m]))
        static.add(TFMessage(transforms=[m]))
    assert time.monotonic() - t0 < 20.0
    assert not moving.values and len(static.values[('base_link', 'tool0')]) <= 2


def test_more_folds_than_captures_is_invalid(world_factory, tmp_path):
    (tmp_path / 'folds.yaml').write_text('session:\n  target_captures: 12\n'
                                         'solver:\n  folds: 15\n')
    w = world_factory(node_over={'profiles_dir': str(tmp_path)})
    res = w.result(w.driver.send(profile='folds')).result
    assert res.error_code == R.INVALID_GOAL and 'target_captures' in res.message


def test_too_few_inliers_maps_to_not_enough_samples(world_factory, monkeypatch):
    from handeye_autopilot.core import quality
    from handeye_autopilot.core.solver import TOO_FEW_CAPTURES, SolveError
    import handeye_autopilot.ros.node as node_mod
    w = world_factory()

    def fail(mode, records, cfg, **kw):
        return quality.Calibration(mode, None, quality.QualityReport(), [],
                                   SolveError(TOO_FEW_CAPTURES, '5 captures left'))
    monkeypatch.setattr(node_mod, 'calibrate', fail)
    res = _finish_run(w)
    assert res.error_code == R.NOT_ENOUGH_SAMPLES and not res.transform_valid


# -- review fixes (M3 xhigh pass 8-9) ------------------------------------------------

def test_recalibration_of_an_attached_camera_is_publishable_after_the_old_one_stops():
    """parent -> camera_link already exists (an earlier result): the node accepts
    it as the mount being replaced; publish_result checks liveness at publish."""
    from handeye_autopilot.ros import tf_tools
    from tf2_msgs.msg import TFMessage
    static = tf_tools.StaticLinks()
    static.add(TFMessage(transforms=[tf_tools.to_msg(np.eye(4), 'camera_link', 'opt')]))
    tree = {'opt': 'camera_link', 'camera_link': 'base_link'}
    sub = tf_tools.camera_subtree(tree, static, 'opt', 'base_link', '')
    assert sub.error == '' and sub.root == 'camera_link'
    from handeye_autopilot.ros.publish_result import check
    assert 'already has a parent' in check(tree, 'base_link', 'camera_link')   # still live


def test_cancel_keeps_the_capture_counts(world_factory):
    w = world_factory()
    d = w.driver
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, 2)
    d.wait(h.cancel_goal_async())
    q = w.result(h).result.quality
    assert q.captures_accepted + q.captures_rejected == 2


def test_preflight_waits_the_full_tf_latency(world_factory):
    """TF arriving 0.4 s late (inside the 0.5 s tf_timeout) at a 10 Hz camera."""
    w = world_factory(cam_over={'stamp_from_clock': True})
    w.robot.delay = 0.4
    time.sleep(0.8)
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=15.0)
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_a_client_waiting_early_gets_the_real_result(world_factory):
    """Request the result BEFORE the run ends: rclpy sends whatever the terminal
    transition carried, so it must carry the real result (not a default one)."""
    w = world_factory()
    d = w.driver
    h = d.send()
    fut = h.get_result_async()                     # waiting from the start
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    run_captures(w, len(w.poses), accepted_target=10)
    res = d.wait(fut, 120.0).result
    assert res.error_code == R.NONE and res.message and res.quality.captures_accepted >= 10


# -- review fixes (M3 xhigh pass 10) ------------------------------------------------

def test_mixed_outcomes_do_not_wedge_the_server(world_factory):
    w = world_factory()
    d = w.driver
    assert _finish_run(w).error_code == R.NONE          # success (info log)
    h = d.send()
    d.wait_phase(FB.PHASE_MANUAL_CAPTURE)
    d.wait(h.cancel_goal_async())
    assert w.result(h).result.error_code == R.CANCELED  # cancel (warning log)
    h = d.send()
    assert h.accepted                                   # not stuck "busy"
    d.wait(h.cancel_goal_async())
    w.result(h)


def test_a_stopped_camera_fails_preflight(world_factory):
    w = world_factory(node_over={'timing.camera_info_timeout': 2.0})
    w.camera.timer.cancel()                             # the stream stops
    time.sleep(1.5)                                     # in-flight frames drain (slow CI)
    res = w.result(w.driver.send()).result
    assert res.error_code == R.NO_CAMERA_INFO and 'fresh images' in res.message


# -- review fixes (M3 xhigh pass 11) ------------------------------------------------

def test_failed_gates_are_not_publishable_by_default(world_factory, tmp_path):
    """A run that completes but fails its quality gates: kept, not publishable."""
    import os

    from handeye_autopilot.ros.publish_result import ResultPublisher
    import yaml
    w = world_factory(node_over={'solver.max_reprojection_rms_px': 1e-6})   # impossible gate
    res = _finish_run(w)
    assert res.error_code == R.NONE and res.transform_valid and not res.passed
    run_dir = os.path.dirname(res.result_file)
    assert not os.path.exists(os.path.join(run_dir, '.publishable'))
    with pytest.raises(SystemExit, match='quality gates'):
        ResultPublisher(res.result_file, listen=0.0)
    node = ResultPublisher(res.result_file, listen=0.0, allow_failed=True)   # explicit
    node.destroy_node()
    assert yaml.safe_load(open(res.result_file))['passed'] is False


# -- review fixes (M3 xhigh pass 12) ------------------------------------------------

def test_sim_reset_with_the_default_synthetic_camera(world_factory):
    """Default camera (images stamped from TF) and in-flight TF across a reset."""
    w = world_factory(sim_time=True)
    w.robot.delay = 0.15
    time.sleep(0.5)
    w.clock.jump(-100.0)
    time.sleep(1.0)
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=20.0)
    w.driver.capture_and_wait()
    assert w.driver.feedback[-1].accepted == 1
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_stopped_camera_on_a_paused_clock_fails_preflight(world_factory):
    w = world_factory(sim_time=True, node_over={'timing.camera_info_timeout': 2.0})
    w.camera.timer.cancel()                   # the stream stops ...
    w.clock.pause()                           # ... and sim time stands still
    time.sleep(0.3)
    res = w.result(w.driver.send()).result
    assert res.error_code == R.NO_CAMERA_INFO and 'fresh images' in res.message


def test_recalibration_after_a_mode_switch_is_accepted():
    """An earlier eye-to-hand result published base_link -> camera_link; an
    eye-in-hand recalibration (parent tool0) treats it as the mount replaced."""
    from handeye_autopilot.ros import tf_tools
    from tf2_msgs.msg import TFMessage
    static = tf_tools.StaticLinks()
    static.add(TFMessage(transforms=[tf_tools.to_msg(np.eye(4), 'camera_link', 'opt')]))
    tree = {'opt': 'camera_link', 'camera_link': 'base_link', 'tool0': 'base_link'}
    sub = tf_tools.camera_subtree(tree, static, 'opt', 'tool0', '', None,
                                  {'base_link', 'tool0'})
    assert sub.error == '' and sub.root == 'camera_link'


# -- review fixes (M3 xhigh pass 13) ------------------------------------------------

def test_short_rewind_then_a_goal(world_factory):
    """A sub-second sim rewind (0.95 s) shortly before a goal, TF in flight."""
    w = world_factory(sim_time=True, node_over={'timing.clock_tolerance': 0.5})
    w.robot.delay = 0.15
    time.sleep(0.5)
    w.clock.jump(-0.95)
    time.sleep(0.25)
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=20.0)
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_verifying_phase_is_active_when_k_fold_starts(world_factory, monkeypatch):
    from handeye_autopilot.core import quality
    w = world_factory()
    seen = []
    real = quality.k_folds

    def spy(*a, **k):
        seen.append(w.node._run.phase)            # the phase at k-fold entry
        return real(*a, **k)
    monkeypatch.setattr(quality, 'k_folds', spy)
    _finish_run(w)
    assert seen == [FB.PHASE_VERIFYING]


def test_a_queued_old_resolution_frame_does_not_abort_preflight(world_factory):
    w = world_factory()
    from sensor_msgs.msg import Image
    old = Image(encoding='mono8', width=4, height=4, step=4, data=bytes(16))
    old.header.frame_id = 'camera_optical_frame'
    old.header.stamp = w.node.get_clock().now().to_msg()
    w.camera.timer.cancel()                       # pause the stream ...
    time.sleep(1.5)                               # in-flight frames drain (slow CI)
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_PREFLIGHT)
    w.node._on_image(old)                         # ... so preflight sees the stale frame
    time.sleep(0.5)
    assert w.node._run.phase == FB.PHASE_PREFLIGHT  # still waiting, not aborted
    w.camera.timer.reset()                        # matching frames resume
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=15.0)
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_new_resolution_image_before_its_camera_info(world_factory):
    """Camera changes resolution: the new image arrives first, its CameraInfo
    150 ms later. Preflight must pin the NEW pair (the unfixed code pinned the
    old CameraInfo and aborted)."""
    from sensor_msgs.msg import Image
    w = world_factory()
    w.camera.timer.cancel()                        # only our injected stream from here
    time.sleep(1.0)                                # (let in-flight 640x480 frames drain)
    new = w.camera.info(w.node.get_clock().now().to_msg())
    new.width, new.height = 320, 240               # the camera switched resolution
    new.k = [300.0, 0.0, 159.5, 0.0, 300.0, 119.5, 0.0, 0.0, 1.0]
    new.p = [300.0, 0.0, 159.5, 0.0, 0.0, 300.0, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_PREFLIGHT)
    stop = threading.Event()

    def stream():                                   # image first, CameraInfo 150 ms later
        sent_info = False
        t0 = time.monotonic()
        while not stop.is_set():
            img = Image(encoding='mono8', width=320, height=240, step=320,
                        data=bytes(320 * 240))
            img.header.frame_id = 'camera_optical_frame'
            img.header.stamp = w.node.get_clock().now().to_msg()
            w.node._on_image(img)
            if not sent_info and time.monotonic() - t0 > 0.15:
                w.node._on_info(new)
                sent_info = True
            time.sleep(0.05)
    t = threading.Thread(target=stream, daemon=True)
    t.start()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=15.0)
    assert w.node._run.info.width == 320           # the new pair was pinned
    stop.set()
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_camera_info_change_during_the_preflight_tf_wait(world_factory):
    """TF arrives 350 ms late; the camera switches resolution while preflight is
    waiting for it. The old pair must not be pinned."""
    from sensor_msgs.msg import Image
    w = world_factory(cam_over={'stamp_from_clock': True})
    w.camera.timer.cancel()
    time.sleep(1.0)
    w.robot.delay = 0.35
    time.sleep(0.5)
    old = w.node._info
    new = w.camera.info(w.node.get_clock().now().to_msg())
    new.width, new.height = 320, 240
    new.k = [300.0, 0.0, 159.5, 0.0, 300.0, 119.5, 0.0, 0.0, 1.0]
    new.p = [300.0, 0.0, 159.5, 0.0, 0.0, 300.0, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    h = w.driver.send()
    w.driver.wait_phase(FB.PHASE_PREFLIGHT)
    stop = threading.Event()

    def stream():
        t0 = time.monotonic()
        switched = False
        while not stop.is_set():
            size = (320, 240) if switched else (old.width, old.height)
            img = Image(encoding='mono8', width=size[0], height=size[1], step=size[0],
                        data=bytes(size[0] * size[1]))
            img.header.frame_id = 'camera_optical_frame'
            img.header.stamp = w.node.get_clock().now().to_msg()
            w.node._on_image(img)
            if not switched and time.monotonic() - t0 > 0.15:     # mid TF wait
                w.node._on_info(new)
                switched = True
            time.sleep(0.05)
    threading.Thread(target=stream, daemon=True).start()
    w.driver.wait_phase(FB.PHASE_MANUAL_CAPTURE, timeout=15.0)
    assert w.node._run.info.width == 320
    stop.set()
    w.driver.wait(h.cancel_goal_async())
    w.result(h)


def test_internal_error_keeps_the_capture_counts(world_factory, monkeypatch):
    w = world_factory()

    def readonly(*a, **k):
        raise PermissionError('read-only output volume')
    monkeypatch.setattr(w.node, '_write_results', readonly)
    res = _finish_run(w)
    assert res.error_code == R.INTERNAL and res.quality.captures_accepted >= 10
