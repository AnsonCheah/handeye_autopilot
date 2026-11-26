"""Each per-capture gate triggers its own rejection code (02-features §D3)."""

import cv2
from handeye_autopilot.core import codes
from handeye_autopilot.core.camera import CameraModel
from handeye_autopilot.core.capture import Frame, process_detection, process_frame
from handeye_autopilot.core.config import CaptureConfig
from handeye_autopilot.core.detection import Detection
from m1a_scenes import SPEC, build
import numpy as np
import pytest

CFG = CaptureConfig()


def outcome(scene_name, cam=None):
    s = build(SPEC['gate_scenes'][scene_name])
    res = process_frame(Frame(s.render(), 0.0, np.eye(4)), s.board, cam or s.camera, CFG)
    return res, s


def test_no_board():
    s = build(SPEC['gate_scenes']['near_border'])
    blank = np.full((s.camera.height, s.camera.width), 128, np.uint8)
    for spec in (s.board, build(SPEC['gate_scenes']['blurry']).board):
        res = process_frame(Frame(blank, 0.0, np.eye(4)), spec, s.camera, CFG)
        assert res.outcome.reject == codes.NO_BOARD


@pytest.mark.parametrize('scene, code', [
    ('few_features', codes.FEW_FEATURES),
    ('blurry', codes.BLURRY),
    ('near_border', codes.NEAR_BORDER),
    ('cropped', codes.NEAR_BORDER),
])
def test_rendered_gate(scene, code):
    res, _ = outcome(scene)
    assert res.outcome.reject == code, (res.outcome.reject, res.outcome.detail)


def test_false_checkerboard_grid_is_rejected_not_misordered():
    sc = SPEC['gate_scenes']['checker_false_grid']
    s = build(sc)
    res = process_frame(Frame(s.render(seed=sc['render_seed']), 0.0, np.eye(4)), s.board,
                        s.camera, CFG)
    assert res.outcome.reject == codes.NO_BOARD, res.outcome.detail
    assert res.outcome.metrics['pattern_agreement'] < CFG.min_pattern_agreement


def test_reprojection_gate_with_wrong_distortion():
    """Rendered through a strongly distorted lens, detected with D = 0: bad fit."""
    s = build(SPEC['gate_scenes']['reprojection'])
    wrong = CameraModel(s.camera.K, np.zeros(5), s.camera.width, s.camera.height)
    res = process_frame(Frame(s.render(), 0.0, np.eye(4)), s.board, wrong, CFG)
    assert res.outcome.reject == codes.REPROJECTION, (res.outcome.reject, res.outcome.detail)


def _subset(scene_name, keep):
    """A real detection with only some corners kept (as if the rest were occluded),
    run through the same path as a live frame."""
    s = build(SPEC['gate_scenes'][scene_name])
    frame = Frame(s.render(), 0.0, np.eye(4))
    res = process_frame(frame, s.board, s.camera, CFG)
    assert res.outcome.accepted, res.outcome.detail
    d = res.detection
    sel = keep(s.board, d)
    det = Detection(d.ids[sel], d.image_points[sel], d.object_points[sel])
    return process_detection(det, frame, s.board, s.camera, CFG).outcome


def test_collinear_gate():
    def one_row(board, d):
        cols = board.inner_corners[0]
        return (d.ids // cols) == 1
    out = _subset('long_board', one_row)
    assert out.reject == codes.COLLINEAR, (out.reject, out.detail)


def test_poor_spread_gate():
    def cluster(board, d):
        cols = board.inner_corners[0]
        c, r = d.ids % cols, d.ids // cols
        return (c < 4) & (r < 3)
    out = _subset('long_board', cluster)
    assert out.metrics['corners'] >= CFG.min_corners
    assert out.reject == codes.POOR_SPREAD, (out.reject, out.detail)


def test_exactly_collinear_corners_are_rejected_before_pnp():
    """Regression: IPPE on exactly collinear points gave NaN poses (Codex M1a review).
    A real image with one corner row left, through the full frame path."""
    res, _ = outcome('one_row')
    assert res.outcome.reject == codes.COLLINEAR, (res.outcome.reject, res.outcome.detail)
    assert res.outcome.metrics['collinearity'] < 1e-6 and res.pose is None


def test_every_capture_code_has_a_test():
    """Guard: codes.ALL is covered by this file, the fixtures or test_capture.py."""
    covered = {codes.NO_BOARD, codes.FEW_FEATURES, codes.BLURRY, codes.NEAR_BORDER,
               codes.REPROJECTION, codes.COLLINEAR, codes.POOR_SPREAD,
               codes.AMBIGUOUS_POSE, codes.MARKER_SUPPORT, codes.TWO_BOARDS,  # fixtures
               codes.CONSENSUS, codes.TF_UNAVAILABLE}                         # capture
    assert covered == set(codes.ALL)


def test_sharpness_is_lower_when_blurred():
    from handeye_autopilot.core.gates import sharpness
    s = build(SPEC['fixtures']['charuco_tilted'])
    img = s.render()
    px = s.camera.project_object(s.camera_T_board, s.board.corner_points)
    assert sharpness(cv2.GaussianBlur(img, (0, 0), 1.5), px) < sharpness(img, px)
