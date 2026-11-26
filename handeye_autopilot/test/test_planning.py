"""Planning pieces (02-features §B2, §C): step checks, filters, selection, ordering."""

import math

from handeye_autopilot.core import bootstrap as bs
from handeye_autopilot.core import planning
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.config import SessionConfig, SolverConfig
from handeye_autopilot.core.render import look_at_board
from m1b_sim import BOARD, CAMERA, truth
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

CFG = SessionConfig()


def board_pose(distance=0.55, tilt=(20, -15), roll=5, offset=(0, 0)):
    w, h = BOARD.size_m
    return look_at_board(np.array([w / 2, h / 2, 0]), distance, *np.radians(tilt),
                         math.radians(roll), offset_px=offset, camera=CAMERA)


@pytest.mark.parametrize('mode', ['eye_to_hand', 'eye_in_hand'])
def test_rotation_check_is_monotonic_and_bounded(mode):
    C = board_pose()
    safe = [bs.rotation_is_safe(mode, C, BOARD, CAMERA, math.radians(a), CFG)
            for a in (0.5, 1, 2, 4, 8, 16, 32)]
    assert safe[0] and not safe[-1]
    assert safe == sorted(safe, reverse=True)          # once unsafe, stays unsafe


def test_larger_lever_bound_allows_smaller_steps():
    C = board_pose()
    steps = []
    for lever in (0.0, 0.3, 1.0):
        c = SessionConfig(lever_max=lever)
        steps.append(bs.safe_step(lambda v, c=c: bs.rotation_is_safe(
            'eye_in_hand', C, BOARD, CAMERA, v, c), math.radians(16), math.radians(0.1)))
    assert steps[0] >= steps[1] >= steps[2] and steps[0] > steps[2]


def test_translation_check():
    C = board_pose()
    assert bs.translation_is_safe(C, BOARD, CAMERA, 0.005, CFG)
    assert not bs.translation_is_safe(C, BOARD, CAMERA, 0.5, CFG)


@pytest.mark.parametrize('mode', ['eye_to_hand', 'eye_in_hand'])
def test_worst_case_check_holds_for_random_true_geometry(mode):
    """Independent of the check's sampling: random tool orientations and lever
    arms inside the bound, real tool steps through the physical chain, sampled
    along the motion. Whenever the check says safe, the board must stay in view."""
    rng = np.random.default_rng(3)
    cfg = SessionConfig(lever_max=0.15)
    w, h = BOARD.size_m
    for trial in range(25):
        C = board_pose(distance=rng.uniform(0.45, 0.7), tilt=rng.uniform(-25, 25, 2),
                       roll=rng.uniform(-40, 40), offset=rng.uniform(-60, 60, 2))
        lever = rng.uniform(-1, 1, 3) * cfg.lever_max          # per tool axis, inside bound
        R_tc = Rotation.random(random_state=rng).as_matrix()    # unknown tool orientation
        if mode == 'eye_to_hand':
            # tool origin at board centre + lever (tool frame), tool axes R_tc in camera
            ref = tf.apply(C, np.array([[w / 2, h / 2, 0]]))[0]
        else:
            ref = np.zeros(3)
        tool_in_cam = tf.make(R_tc, ref + R_tc @ lever)
        for kind in ('rotation', 'translation'):
            for axis in range(3):
                start = math.radians(12) if kind == 'rotation' else 0.06
                if kind == 'rotation':
                    v = bs.safe_step(lambda x, C=C: bs.rotation_is_safe(
                        mode, C, BOARD, CAMERA, x, cfg), start, start / 16)
                else:
                    v = bs.safe_step(lambda x, C=C: bs.translation_is_safe(
                        C, BOARD, CAMERA, x, cfg), start, start / 16)
                for frac in np.linspace(0, 1, 6)[1:]:
                    for sign in (1, -1):
                        D = bs.tool_step(kind, axis, sign * v * frac)
                        motion = tool_in_cam @ D @ tf.inv(tool_in_cam)    # in camera frame
                        C2 = motion @ C if mode == 'eye_to_hand' else tf.inv(motion) @ C
                        px = CAMERA.project_object(C2, BOARD.outline_samples(12))
                        assert px.min() >= 0 and px[:, 0].max() < CAMERA.width and \
                            px[:, 1].max() < CAMERA.height, (trial, kind, axis, v)


def test_pivot_origin_by_mode():
    C = board_pose()
    w, h = BOARD.size_m
    np.testing.assert_allclose(bs.pivot_origin('eye_to_hand', C, BOARD),
                               tf.apply(C, np.array([[w / 2, h / 2, 0]]))[0])
    np.testing.assert_allclose(bs.pivot_origin('eye_in_hand', C, BOARD), 0)


def test_tool_step():
    T = bs.tool_step('rotation', 2, math.radians(10))
    assert math.degrees(tf.rotation_angle(T, np.eye(4))) == pytest.approx(10)
    np.testing.assert_allclose(bs.tool_step('translation', 1, 0.02)[:3, 3], [0, 0.02, 0])


@pytest.mark.parametrize('C, reason', [
    (board_pose(tilt=(180, 0)), planning.BACK_FACING),
    (board_pose(distance=3.0), planning.COVERAGE),
    (board_pose(offset=(300, 0)), planning.NOT_VISIBLE),
])
def test_visibility_reasons(C, reason):
    assert planning.visibility(C, CAMERA, BOARD, CFG)[0] == reason


def test_visible_pose_passes():
    assert planning.visibility(board_pose(), CAMERA, BOARD, CFG)[0] is None


def test_limits():
    seed = tf.make(np.eye(3), [0.5, 0, 0.4])
    assert planning.limits(seed, seed, CFG)[0] is None
    far = tf.make(np.eye(3), [1.5, 0, 0.4])
    assert planning.limits(far, seed, CFG)[0] == planning.OUTSIDE_ENVELOPE
    turned = tf.make(Rotation.from_euler('x', 90, degrees=True), [0.5, 0, 0.4])
    assert planning.limits(turned, seed, CFG)[0] == planning.ORIENTATION_LIMIT
    boxed = SessionConfig(box_enabled=True, box_xyz=(0, 0, 0), box_size=(0.8, 2, 2))
    assert planning.limits(seed, seed, boxed)[0] == planning.OUTSIDE_BOX


def test_selection_spreads_rotations_more_than_the_pool():
    rng = np.random.default_rng(0)
    pool = [planning.Candidate(np.eye(4), tf.make(
        Rotation.from_rotvec(rng.normal(0, 0.3, 3)), [0, 0, 0])) for _ in range(200)]
    chosen = planning.select('eye_in_hand', pool, [], SessionConfig(target_captures=10,
                                                                     reserve_factor=1.0))

    def min_pairwise(cs):
        return min(tf.rotation_angle(a.tool_pose, b.tool_pose)
                   for i, a in enumerate(cs) for b in cs[i + 1:])
    assert min_pairwise(chosen) > 3 * min_pairwise(pool[:10])


def test_selection_prefers_new_axes():
    """Same angle, different axis: the novel axis wins over a repeat of a used one."""
    a = math.radians(20)
    existing = [np.eye(4), tf.make(Rotation.from_rotvec([a, 0, 0]), [0, 0, 0])]
    same_axis = planning.Candidate(np.eye(4), tf.make(Rotation.from_rotvec([-a, 0, 0]),
                                                      [0, 0, 0]))
    new_axis = planning.Candidate(np.eye(4), tf.make(Rotation.from_rotvec([0, a, 0]),
                                                     [0, 0, 0]))
    out = planning.select('eye_in_hand', [same_axis, new_axis], existing, CFG, count=1)
    assert out == [new_axis]


def test_half_turn_relative_rotation_has_a_real_axis():
    """A 180-degree turn about a used axis is not 'novel' (Codex M2 review)."""
    a = math.radians(20)
    existing = [np.eye(4), tf.make(Rotation.from_rotvec([a, 0, 0]), [0, 0, 0])]
    half_turn_x = planning.Candidate(np.eye(4), tf.make(Rotation.from_rotvec([math.pi, 0, 0]),
                                                        [0, 0, 0]))
    ang, ax = planning._relative(np.eye(3)[None], half_turn_x.tool_pose[:3, :3][None])
    assert ang[0] == pytest.approx(math.pi) and abs(ax[0] @ [1, 0, 0]) == pytest.approx(1)
    novel = planning.Candidate(np.eye(4), tf.make(Rotation.from_rotvec([0, 2.5, 0]),
                                                  [0, 0, 0]))
    out = planning.select('eye_in_hand', [half_turn_x, novel], existing, CFG, count=1)
    assert out == [novel]


@pytest.mark.parametrize('dictionary', ['DICT_APRILTAG_36h11', '5X5_100', 'DICT_4X4_50'])
def test_marker_module_reads_the_dictionary(dictionary):
    from handeye_autopilot.core.boards import BoardSpec
    b = BoardSpec('charuco', 5, 4, 0.04, 0.03, dictionary)
    C = board_pose()
    module = planning.marker_module_px(C, CAMERA, b)
    bits = {'DICT_APRILTAG_36h11': 6, '5X5_100': 5, 'DICT_4X4_50': 4}[dictionary] + 2
    # roughly the projected marker side over the cells
    side = 0.03 * CAMERA.K[0, 0] / 0.55
    assert 0.5 * side / bits < module < 1.2 * side / bits
    planning.visibility(C, CAMERA, b, CFG)            # no crash for any dictionary


def test_uncertainty_term_widens_the_margin():
    C = board_pose(offset=(120, 0))
    m = planning.visibility(C, CAMERA, BOARD, CFG, uncertainty_px=0.0)[0]
    big = planning.visibility(C, CAMERA, BOARD, CFG, uncertainty_px=200.0)[0]
    assert m is None and big == planning.NOT_VISIBLE


def test_posed_box():
    c = SessionConfig(box_enabled=True, box_xyz=(0.5, 0, 0.4),
                      box_rpy=(0, 0, math.radians(45)), box_size=(0.2, 0.2, 0.2))
    inside = tf.make(np.eye(3), [0.5, 0.0, 0.4])
    corner = tf.make(np.eye(3), [0.58, 0.08, 0.4])          # inside an axis-aligned box only
    assert planning.limits(inside, inside, c)[0] is None
    assert planning.limits(corner, inside, c)[0] == planning.OUTSIDE_BOX
    pts = np.array([[0.5, 0.2, 0.4]])                       # carried board corner outside
    on = SessionConfig(**{**c.to_dict(), 'box_check_moving_object': True})
    assert planning.limits(inside, inside, c, pts)[0] is None
    assert planning.limits(inside, inside, on, pts)[0] == planning.OUTSIDE_BOX


def test_ordering_is_a_nearest_neighbour_tour():
    cs = [planning.Candidate(np.eye(4), joints=np.array([x, 0.0])) for x in (5, 1, 3, 2, 4)]
    out = planning.order(cs, np.array([0.0, 0.0]))
    assert [c.joints[0] for c in out] == [1, 2, 3, 4, 5]


def test_plan_records_rejections_and_meets_information():
    X, Z = truth('eye_to_hand')
    seed_C = board_pose()
    seed_tool = X @ seed_C @ tf.inv(Z)

    def ik(G):
        return planning.IKResult(np.r_[G[:3, 3], Rotation.from_matrix(G[:3, :3]).as_rotvec()])
    p = planning.plan('eye_to_hand', X, Z, CAMERA, BOARD, seed_tool, None, [], ik, CFG,
                      SolverConfig())
    assert not p.weak and p.information >= CFG.min_plan_singular_value
    assert len(p.poses) >= p.target >= CFG.target_captures
    assert len(p.poses) <= p.target + math.ceil(p.target * (CFG.reserve_factor - 1)) + 1
    assert sum(p.rejections.values()) + len(p.poses) <= CFG.candidates
    assert p.rejections[planning.NOT_VISIBLE] > 0
    for c in p.poses:                       # every planned pose shows the board
        assert planning.visibility(c.camera_T_board, CAMERA, BOARD, CFG)[0] is None


def test_ik_failures_are_recorded():
    X, Z = truth('eye_to_hand')
    seed_tool = X @ board_pose() @ tf.inv(Z)
    p = planning.plan('eye_to_hand', X, Z, CAMERA, BOARD, seed_tool, None, [],
                      lambda G: planning.IKResult(None, 'NO_IK_SOLUTION'), CFG, SolverConfig())
    assert p.poses == [] and p.weak and p.rejections[planning.IK_FAILED] > 0
