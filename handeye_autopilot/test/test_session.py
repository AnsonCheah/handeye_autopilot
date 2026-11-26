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
    assert r.plan_weak and r.plan.weak
    assert r.plan.information < c.min_plan_singular_value
    # the run still finishes; here the final solve rejects the tiny rotations
    assert r.failure == 'degenerate_rotation' or 'weak' in r.message
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
        s.on_motion(True)          # first action is a capture, not a move
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
