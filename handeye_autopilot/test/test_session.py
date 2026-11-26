"""M2 done-when: full pure-Python sessions, probe halving, spares, weak plan,
too-small L_max (spec/m2.yaml)."""

import math

from handeye_autopilot.core import codes, session
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.config import SessionConfig
from handeye_autopilot.core.session import Move
from m2_sim import SPEC, FakeIK, run
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

MODES = ['eye_to_hand', 'eye_in_hand']
TOL = SPEC['tolerance']


def cfg(**over):
    return SessionConfig(**{**SessionConfig().to_dict(), **SPEC['session'],
                            'frames_per_capture': 1, **over})


@pytest.mark.parametrize('mode', MODES)
def test_full_session_recovers_truth_and_passes(mode):
    r, X, Z = run(mode)
    assert r.failure is None, r.message
    assert r.elapsed < SPEC['deadline_s']
    cal = r.calibration
    assert cal.passed, [g.name for g in cal.report.gates if g.gated and not g.passed]
    assert tf.translation_distance(cal.solution.X, X) <= TOL['translation_m']
    assert math.degrees(tf.rotation_angle(cal.solution.X, X)) <= TOL['rotation_deg']
    assert r.bootstrap_rounds >= 1 and not r.plan_weak
    planned = sum(rec.accepted for rec, why in zip(r.records, r.capture_purposes, strict=True)
                  if why == 'plan')
    assert planned >= r.plan.target >= SessionConfig().target_captures


def test_consensus_frames_end_to_end():
    r, *_ = run('eye_to_hand', frames=3)
    assert r.failure is None and r.calibration.passed
    used = [rec.metrics['frames_used'] for rec in r.records if rec.accepted]
    assert used and all(u == 3 for u in used)


@pytest.mark.parametrize('mode', MODES)
def test_lost_board_halves_the_step_and_recovers(mode):
    log = []
    r, *_ = run(mode, lose={1}, log=log)          # capture 1 = first bootstrap move
    assert r.failure is None, r.message
    assert r.records[1].reject == codes.NO_BOARD
    moves = [a for a in log if isinstance(a, Move)]
    steps = [a for a in moves if a.purpose == 'bootstrap']
    first, retry = steps[0], steps[1]
    between = moves[next(i for i, m in enumerate(moves) if m is first) + 1]
    assert between.purpose == 'bootstrap: to the seed'      # back to the last good pose
    seed_tool = r.records[0].tool_pose
    # retry is half of the first step about the same axis
    a1 = tf.rotation_angle(first.tool_pose, seed_tool)
    a2 = tf.rotation_angle(retry.tool_pose, seed_tool)
    assert a2 == pytest.approx(a1 / 2, rel=0.02)


@pytest.mark.parametrize('mode', MODES)
def test_too_small_lever_bound_fails_with_board_lost(mode):
    sp = SPEC['lever_too_small'][mode]
    c = cfg(lever_max=sp['lever_max'], first_rotation_step=math.radians(sp['first_rotation_deg']),
            max_halvings=sp['max_halvings'], envelope_radius=10.0)
    r, *_ = run(mode, session_cfg=c, true_lever_scale=sp['true_lever_scale'],
                ik=FakeIK(0.0, 20.0))
    assert r.failure == session.BOARD_LOST
    assert 'L_max' in r.message and r.calibration is None


def test_motions_that_never_started_are_replaced_by_spares():
    def fail(index, purpose):
        return purpose == 'plan' and index % 4 == 0      # every 4th planned motion
    r, *_ = run('eye_in_hand', fail_move=fail)
    assert r.failure is None, r.message
    assert r.skipped_moves >= 3
    planned = sum(rec.accepted for rec, why in zip(r.records, r.capture_purposes, strict=True)
                  if why == 'plan')
    assert planned == r.plan.target
    assert len(r.plan.poses) > r.plan.target                           # spares existed


def test_failed_execution_stops_the_session():
    def fail(index, purpose):
        return session.MOTION_FAILED if purpose == 'plan' else None
    r, *_ = run('eye_to_hand', fail_move=fail)
    assert r.failure == session.MOTION_FAILED_REASON and r.calibration is None


def test_failed_recovery_stops_the_session():
    def fail(index, purpose):
        # the return to the seed after the lost board (the 3rd move) fails
        return index >= 3 and purpose == 'bootstrap: to the seed'
    r, *_ = run('eye_to_hand', lose={1}, fail_move=fail)
    assert r.failure == session.RECOVERY_FAILED


def test_empty_capture_is_a_recorded_rejection():
    from handeye_autopilot.core.session import Capture, Finished
    from m1b_sim import BOARD, CAMERA
    s = session.Session('eye_to_hand', BOARD, CAMERA, FakeIK(0, 2))
    assert isinstance(s.next_action(), Capture)
    s.on_capture([])                                     # camera timeout on the seed
    a = s.next_action()
    assert isinstance(a, Finished) and a.result.failure == session.SEED_REJECTED
    assert 'no frames' in a.result.message


def test_weak_plan_is_reported():
    w = SPEC['weak_plan']
    c = cfg(envelope_radius=w['envelope_radius'],
            max_orientation_change=math.radians(w['max_orientation_change_deg']),
            max_bootstrap_rounds=w['max_bootstrap_rounds'])
    r, *_ = run('eye_to_hand', session_cfg=c)
    assert r.plan_weak and r.plan.weak and r.plan.advice
    assert r.plan.target < c.target_captures or \
        r.plan.information < c.min_plan_singular_value
    # reported, and still executed: every planned pose was attempted and captured
    planned = sum(1 for why in r.capture_purposes if why == 'plan')
    assert planned == r.plan.target > 0
    assert 'weak' in r.message
    assert r.plan.rejections['outside_envelope'] + r.plan.rejections['orientation_limit'] > 0


def test_face_on_seed_is_rejected_with_advice():
    r, *_ = run('eye_to_hand', seed_spec={'distance': 0.6, 'tilt': [2, 1], 'roll': 0,
                                          'offset': [0, 0]})
    assert r.failure == session.SEED_REJECTED
    assert 'tilt it at least' in r.message or 'ambiguous' in r.message


def test_seed_without_board_is_rejected():
    r, *_ = run('eye_in_hand', lose={0})
    assert r.failure == session.SEED_REJECTED and 'no_board' in r.message


def test_driver_protocol_is_enforced():
    s = session.Session('eye_to_hand', *(__import__('m1b_sim').BOARD,
                                         __import__('m1b_sim').CAMERA), FakeIK(0, 2))
    with pytest.raises(RuntimeError, match='Capture'):
        s.on_motion('ok')          # first action is a capture, not a move
    for bare in (True, False, 'false', None):
        with pytest.raises(ValueError):          # only the explicit outcomes are accepted
            s.on_motion(bare)
    with pytest.raises(ValueError):
        session.Session('sideways', __import__('m1b_sim').BOARD,
                        __import__('m1b_sim').CAMERA, FakeIK(0, 2))
    assert np.isfinite(SessionConfig().lever_max)


def test_only_planned_moves_are_free_joint_goals():
    """Bootstrap, centring and recovery moves must follow the checked straight
    tool path (Cartesian); planned poses are plain joint goals."""
    log = []
    r, *_ = run('eye_to_hand', lose={1}, log=log)
    assert r.failure is None, r.message
    moves = [a for a in log if isinstance(a, Move)]
    assert {m.purpose for m in moves} >= {'plan', 'bootstrap', 'bootstrap: to the seed',
                                          'centring'}
    for m in moves:
        assert m.cartesian == (m.purpose != 'plan'), m.purpose


def test_centring_never_starts_from_a_view_the_robot_is_not_at():
    """If the last bootstrap move did not happen, the robot is at the seed, not at
    the last accepted view: centring works from the seed view (no redundant
    return move, since the robot is already there)."""
    log = []
    run('eye_to_hand', log=log)
    moves = [a for a in log if isinstance(a, Move)]
    last_boot = max(i for i, m in enumerate(moves, 1) if m.purpose == 'bootstrap')

    log2 = []
    r, *_ = run('eye_to_hand', log=log2,
                fail_move=lambda i, why: i == last_boot and why == 'bootstrap')
    assert r.failure is None, r.message
    moves2 = [a for a in log2 if isinstance(a, Move)]
    k = next(i for i, m in enumerate(moves2) if m.purpose.startswith('centring'))
    assert moves2[k].purpose == 'centring'          # no move to where it already is
    seed_tool = r.records[0].tool_pose
    boot = [rec for rec, why in zip(r.records, r.capture_purposes, strict=True)
            if why == 'bootstrap' and rec.accepted][-1].tool_pose
    first = moves2[k].tool_pose
    # the first increment is a small step from the seed view, not from `boot`
    assert tf.translation_distance(first, seed_tool) < tf.translation_distance(first, boot)
    assert tf.rotation_angle(first, seed_tool) < tf.rotation_angle(first, boot)


def test_no_recovery_move_to_where_the_robot_already_is():
    """A skipped step leaves the robot at the seed: the next step's return to the
    seed is not sent (a not_started reply to it would wrongly end the run)."""
    log = []
    r, *_ = run('eye_to_hand', log=log,
                fail_move=lambda i, why: i == 1 and why == 'bootstrap')
    assert r.failure is None, r.message
    moves = [a for a in log if isinstance(a, Move)]
    assert moves[0].purpose == 'bootstrap' and moves[1].purpose == 'bootstrap'


def test_seed_outside_the_safety_box_is_rejected():
    far = cfg(box_enabled=True, box_xyz=(5.0, 5.0, 5.0), box_size=(0.5, 0.5, 0.5))
    log = []
    r, *_ = run('eye_to_hand', session_cfg=far, log=log)
    assert r.failure == session.SEED_REJECTED and 'safety box' in r.message
    assert not [a for a in log if isinstance(a, Move)]          # nothing was sent


def test_checked_moves_carry_dense_waypoints_on_the_straight_path():
    log = []
    r, *_ = run('eye_in_hand', log=log)
    assert r.failure is None, r.message
    c = SessionConfig()
    for m in (a for a in log if isinstance(a, Move) and a.cartesian):
        assert m.waypoints and np.allclose(m.waypoints[-1], m.tool_pose)
        for a, b in zip(m.waypoints, m.waypoints[1:], strict=False):
            assert tf.rotation_angle(a, b) <= c.waypoint_max_angle + 1e-9
            assert tf.translation_distance(a, b) <= c.waypoint_max_step + 1e-9
    for m in (a for a in log if isinstance(a, Move) and not a.cartesian):
        assert m.waypoints == []


def test_bootstrap_growth_needs_both_x_and_z_settled():
    from handeye_autopilot.core.session import settled_estimate
    from handeye_autopilot.core.solver import Solution
    c = SessionConfig()
    a = Solution.__new__(Solution)
    b = Solution.__new__(Solution)
    a.X, a.Z = np.eye(4), np.eye(4)
    b.X, b.Z = np.eye(4), tf.make(np.eye(3), [0.05, 0, 0])   # Z moved 5 cm
    assert settled_estimate(a, a, c)
    assert not settled_estimate(b, a, c)


def _eih_box_session(**over):
    """eye_in_hand session with a known estimate: camera 200 mm along tool z."""
    from m1b_sim import BOARD, CAMERA
    c = cfg(box_enabled=True, box_check_moving_object=True, box_xyz=(0.0, 0.0, 0.0),
            envelope_radius=1.0, **over)
    s = session.Session('eye_in_hand', BOARD, CAMERA, FakeIK(0, 3), cfg=c)
    s._st.X, s._st.Z = tf.make(np.eye(3), [0.0, 0.0, 0.2]), np.eye(4)
    s._st.seed_tool = np.eye(4)
    return s


def _swing(deg):
    return tf.make(Rotation.from_euler('x', deg, degrees=True), np.zeros(3))


def _refused(s, start, target):
    s._st.at_pose = start
    gen = s._move(target, 'centring')
    try:
        next(gen)
    except StopIteration as stop:
        return stop.value is False
    return False


def test_skip_reasons_keep_the_ik_error():
    s = _eih_box_session(box_size=(2.0, 2.0, 2.0))
    from handeye_autopilot.core.planning import IKResult
    s.ik = lambda G: IKResult(None, 'TIMED_OUT')
    s._st.at_pose = np.eye(4)
    assert _refused(s, np.eye(4), _swing(1))
    assert s._st.skip_reasons == {'ik_failed: TIMED_OUT': 1}


def test_spares_are_taken_nearest_to_the_current_joints():
    log = []
    plan_moves = []

    def fail(index, purpose):
        if purpose == 'plan':
            plan_moves.append(index)
            return len(plan_moves) <= 6          # the first six planned poses never start
        return None
    r, *_ = run('eye_to_hand', log=log, fail_move=fail)
    assert r.failure is None, r.message
    moves = [a for a in log if isinstance(a, Move)]
    target = r.plan.target
    spare_ids = {id(c) for c in r.plan.poses[target:]}
    plan = [m for m in moves if m.purpose == 'plan']
    # the first spare is sent after all main poses; it must be the spare nearest
    # the joints the robot was at (the last main pose that ran)
    k = next(i for i, m in enumerate(plan) if any(
        np.array_equal(m.joints, c.joints) for c in r.plan.poses[target:]))
    prev = plan[k - 1].joints
    best = min(r.plan.poses[target:], key=lambda c: np.linalg.norm(c.joints - prev))
    assert np.array_equal(plan[k].joints, best.joints) and spare_ids


def test_orientation_limit_holds_between_waypoints():
    """+179.6 -> -179.6 deg about z: both ends are inside a 179.8 deg limit, the
    short path passes 180 deg in between; the move must be refused."""
    s = _eih_box_session(box_size=(4.0, 4.0, 4.0), max_orientation_change=math.radians(179.8))

    def yaw(d):
        return tf.make(Rotation.from_euler('z', d, degrees=True), np.zeros(3))
    for d in (179.6, -179.6):
        assert tf.rotation_angle(yaw(d), np.eye(4)) < math.radians(179.8)
    assert _refused(s, yaw(179.6), yaw(-179.6))
    assert _refused(s, yaw(170), yaw(171)) is False          # well inside: allowed


def test_not_started_reply_updates_the_joints():
    s = _eih_box_session(box_size=(4.0, 4.0, 4.0))
    s._st.at_pose = np.eye(4)
    gen = s._move(_swing(1), 'centring')
    next(gen)
    fresh = np.arange(7.0)
    with pytest.raises(StopIteration):
        gen.send(('not_started', fresh, None))
    assert np.array_equal(s._st.joints, fresh)


def test_carried_object_box_check_uses_the_lever_bound_not_the_estimate():
    """The estimate says the camera sits on the tool (offset 0), but L_max = 0.1 m:
    near a box face the lever ball pokes out, so the move is refused; a rough
    estimate must never relax the box."""
    s = _eih_box_session(box_size=(4.0, 4.0, 2 * 0.25), lever_max=0.1)
    s._st.X = np.eye(4)                                      # (wrong) estimate: no offset
    r = math.sqrt(3) * 0.1
    near = tf.make(np.eye(3), [0.0, 0.0, 0.25 - r + 0.01])
    far = tf.make(np.eye(3), [0.0, 0.0, 0.25 - r - 0.01])
    assert _refused(s, np.eye(4), near)
    assert _refused(s, np.eye(4), far) is False


def test_path_start_is_checked():
    """A Cartesian move from a pose whose lever ball already pokes out of the box
    is refused."""
    s = _eih_box_session(box_size=(4.0, 4.0, 2 * 0.25), lever_max=0.1)
    r = math.sqrt(3) * 0.1
    start = tf.make(np.eye(3), [0.0, 0.0, 0.25 - r + 0.01])
    assert _refused(s, start, tf.make(np.eye(3), [0.0, 0.0, 0.0]))


@pytest.mark.parametrize('outcome', ['ok', 'not_started'])
def test_measured_pose_outside_the_limits_stops_the_run(outcome):
    s = _eih_box_session(box_size=(4.0, 4.0, 2 * 0.25), lever_max=0.05)
    s._st.at_pose = np.eye(4)
    gen = s._move(_swing(1), 'centring')
    assert isinstance(next(gen), Move)
    drifted = tf.make(np.eye(3), [0.0, 0.0, 0.2])           # ball now pokes out
    with pytest.raises(session._Abort, match='measured tool pose'):
        gen.send((outcome, None, drifted))


def test_capture_pose_outside_the_limits_stops_the_run(monkeypatch):
    """The motion reply said inside, the capture's TF says outside: the capture
    is the freshest measurement and wins."""
    s = _eih_box_session(box_size=(4.0, 4.0, 2 * 0.25), lever_max=0.05)
    s._st.at_pose = np.eye(4)
    outside = tf.make(np.eye(3), [0.0, 0.0, 0.2])
    rec = session.CaptureRecord(0, 0.0, s.camera, np.zeros(0, np.int32), None, None,
                                outside, None, None, None, False, codes.NO_BOARD, '')
    monkeypatch.setattr(session, 'capture', lambda *a, **k: rec)
    gen = s._capture('centring')
    next(gen)
    with pytest.raises(session._Abort, match='at capture'):
        gen.send(([object()], None))
