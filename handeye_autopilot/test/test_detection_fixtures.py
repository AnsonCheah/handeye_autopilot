"""Stored fixture images through detection + gates (03-milestones M1a done-when).

The same PNGs run on OpenCV 4.6 (Jazzy) and 4.10 (Lyrical): IDs and board
coordinates must equal values computed here independently of the detector,
and image corners must be within the committed tolerance of the truth.
"""

import math

import cv2
from handeye_autopilot.core.capture import Frame, process_frame
from handeye_autopilot.core.config import CaptureConfig
from handeye_autopilot.core.gates import ambiguity
from handeye_autopilot.core.pose import BoardPose, PoseSolution
from handeye_autopilot.core.render import look_at_board
from m1a_scenes import SPEC, board, board_centre, build, camera, fixture_path, truth
import numpy as np
import pytest

TRUTH = truth()['fixtures']
TOL = SPEC['tolerances']
FIX = SPEC['fixtures']


def run(name):
    scene = FIX[name]
    img = cv2.imread(str(fixture_path(name)), cv2.IMREAD_GRAYSCALE)
    assert img is not None, f'missing fixture {name}'
    spec, cam = board(scene['board']), camera(scene['camera'])
    res = process_frame(Frame(img, 0.0, np.eye(4)), spec, cam, CaptureConfig())
    return res, spec, cam, np.array(TRUTH[name]['camera_T_board'])


def expected_coordinates(spec, ids):
    """Board-frame corner positions from the documented convention (boards.py)."""
    cols = spec.squares_x - 1
    sq = spec.square_length
    return np.stack([(ids % cols + 1) * sq, (ids // cols + 1) * sq, np.zeros(len(ids))], 1)


def assert_matches_truth(res, spec, cam, T, all_ids, tol=TOL):
    det = res.detection
    n_total = (spec.squares_x - 1) * (spec.squares_y - 1)
    if all_ids:
        np.testing.assert_array_equal(det.ids, np.arange(n_total))
    assert len(det.ids) > 0 and np.all(np.diff(det.ids) > 0)
    np.testing.assert_allclose(det.object_points, expected_coordinates(spec, det.ids),
                               atol=1e-12)
    err = np.linalg.norm(det.image_points - cam.project_object(T, det.object_points), axis=1)
    assert err.max() <= tol['corner_max_px'], err.max()
    assert err.mean() <= tol['corner_mean_px'], err.mean()


@pytest.mark.parametrize('name', [n for n, f in FIX.items() if f['expect'] == 'accepted'])
def test_accepted_fixtures(name):
    res, spec, cam, T = run(name)
    assert res.outcome.accepted, (res.outcome.reject, res.outcome.detail)
    assert_matches_truth(res, spec, cam, T, FIX[name].get('expect_ids') == 'all')


@pytest.mark.parametrize('name', [n for n, f in FIX.items() if 'same_as' in f])
def test_strays_do_not_move_any_corner(name):
    """With the stray masked, corners equal the clean view's corners pixel for pixel."""
    res, *_ = run(name)
    clean, *_ = run(FIX[name]['same_as'])
    assert res.outcome.accepted
    assert res.outcome.metrics['duplicate_markers'] >= 2
    np.testing.assert_array_equal(res.detection.ids, clean.detection.ids)
    np.testing.assert_allclose(res.detection.image_points, clean.detection.image_points,
                               atol=0.02)


@pytest.mark.parametrize('base', ['checker_tilted', 'checker_swapped'])
def test_checkerboard_order_follows_the_board_through_180_degrees(base):
    a, spec, cam, Ta = run(base)
    b, _, _, Tb = run(base.replace('tilted', 'rot180') if 'tilted' in base else base + '_rot180')
    assert_matches_truth(a, spec, cam, Ta, True)
    assert_matches_truth(b, spec, cam, Tb, True)
    assert np.linalg.norm(a.detection.image_points[0] - b.detection.image_points[0]) > 100


@pytest.mark.parametrize('name', [n for n, f in FIX.items() if f['expect'] != 'accepted'])
def test_rejected_fixtures(name):
    res, *_ = run(name)
    assert res.outcome.reject == FIX[name]['expect'], (res.outcome.reject, res.outcome.detail)


def test_masking_branches_are_the_ones_claimed():
    """Each masking fixture reaches the step it is meant to exercise (§D4)."""
    m = run('stray_rejected')[0].outcome.metrics
    assert 'inlier_markers' not in m                         # rejected before any fit
    m = run('marker_cluster')[0].outcome.metrics
    assert m['inlier_markers'] >= CaptureConfig().min_markers  # fit succeeded...
    assert m['inlier_marker_hull_fraction'] < CaptureConfig().min_marker_hull_fraction
    m = run('two_boards')[0].outcome.metrics
    assert m['fit_used_duplicates'] == 1 and m['second_group_markers'] > 0
    m = run('two_boards_partial')[0].outcome.metrics
    assert m['fit_used_duplicates'] == 0 and m['second_group_markers'] > 0
    m = run('stray_duplicates')[0].outcome.metrics
    assert m['second_group_markers'] == 0                    # incoherent strays: one board


def test_exact_duplicate_detections_are_not_two_boards():
    """Every marker reported twice at the same pixels is one board, not two."""
    from handeye_autopilot.core import cv_compat
    from handeye_autopilot.core.detection import mask_markers
    scene = FIX['charuco_tilted']
    img = cv2.imread(str(fixture_path('charuco_tilted')), cv2.IMREAD_GRAYSCALE)
    spec, cam = board(scene['board']), camera(scene['camera'])
    mc, mi = cv_compat.detect_markers(img, cv_compat.get_dictionary(spec.dictionary))
    ids, obj = cv_compat.marker_object_points(spec.opencv_board)
    keep, reject = mask_markers(np.concatenate([mc, mc]), np.concatenate([mi, mi]), ids, obj,
                                spec, cam, CaptureConfig(), {})
    assert reject is None
    assert keep.sum() == len(mi)


def test_ambiguity_pair_meets_the_conditions_it_claims():
    """§D3: the ambiguous fixture meets BOTH conditions; the face-on one fails one."""
    cfg = CaptureConfig()
    m = run('ambiguous')[0].outcome.metrics
    assert m['error_gap_px'] < cfg.ambiguity_k * cfg.sigma_px
    assert m['pose_rotation_gap'] > cfg.ambiguity_min_rotation
    ok = run('faceon_unambiguous')[0]
    m = ok.outcome.metrics
    assert ok.outcome.accepted
    assert not (m['error_gap_px'] < cfg.ambiguity_k * cfg.sigma_px
                and m['pose_rotation_gap'] > cfg.ambiguity_min_rotation)


def test_ambiguity_translation_alternative():
    """Poses with the same rotation but far apart in translation are also ambiguous."""
    cfg = CaptureConfig()
    T = np.eye(4)
    T[:3, 3] = [0, 0, 1.0]
    T2 = T.copy()
    T2[2, 3] = 1.0 + 3 * cfg.ambiguity_min_translation_frac
    pose = BoardPose(PoseSolution(T, 0.1, 0.2, 0.1),
                     [PoseSolution(T, 0.1, 0.2, 0.1), PoseSolution(T2, 0.15, 0.3, 0.15)])
    out = ambiguity(pose, cfg)
    assert out['pose_rotation_gap'] < cfg.ambiguity_min_rotation
    assert out['ambiguous'] == 1.0


def test_pose_sweep():
    """Seeded random views in the committed ranges (spec `sweep`)."""
    sw = SPEC['sweep']
    rng = np.random.default_rng(sw['seed'])
    cam = camera(sw['camera'])
    results = []
    for k in range(sw['views']):
        spec_name = sw['boards'][k % len(sw['boards'])]
        spec = board(spec_name)
        scene = {'camera': sw['camera'], 'board': spec_name,
                 'distance': rng.uniform(*sw['distance']),
                 'tilt': list(rng.uniform(*sw['tilt'], 2)), 'roll': rng.uniform(*sw['roll']),
                 'offset': list(rng.uniform(*sw['offset'], 2))}
        s = build(scene)
        T = look_at_board(board_centre(spec), scene['distance'],
                          *(math.radians(a) for a in (*scene['tilt'], scene['roll'])),
                          offset_px=tuple(scene['offset']), camera=cam)
        res = process_frame(Frame(s.render(seed=1000 + k), 0.0, np.eye(4)), spec, cam,
                            CaptureConfig())
        if res.outcome.accepted:
            assert_matches_truth(res, spec, cam, T, False, tol=sw)
        else:
            assert res.outcome.reject in sw['allowed_rejects'], (k, scene, res.outcome.detail)
        results.append(res.outcome.accepted)
    assert np.mean(results) >= sw['min_accepted_fraction'], results
