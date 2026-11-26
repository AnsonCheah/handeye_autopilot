"""Board definitions, printable textures and the board-image CLI (02-features §J)."""

import cv2
from handeye_autopilot.cli import board_image
from handeye_autopilot.core import cv_compat
from handeye_autopilot.core.boards import BoardSpec, BoardSpecError, board_texture
from handeye_autopilot.core.detection import canonical_checkerboard_order
import numpy as np
import pytest

CHARUCO = dict(kind='charuco', squares_x=7, squares_y=5, square_length=0.04,
               marker_length=0.03, dictionary='DICT_5X5_100')


def test_checkerboard_needs_four_squares_per_side():
    with pytest.raises(BoardSpecError, match='at least 4 squares'):
        BoardSpec('checkerboard', 3, 4, 0.03)


def test_symmetric_checkerboard_rejected_with_advice():
    with pytest.raises(BoardSpecError, match='symmetric.*ChArUco'):
        BoardSpec('checkerboard', 8, 6, 0.03)
    BoardSpec('checkerboard', 7, 6, 0.03)  # odd x even is fine


def test_yaml_uses_square_counts_not_inner_corners():
    with pytest.raises(BoardSpecError, match='square counts'):
        BoardSpec.from_dict({'kind': 'checkerboard', 'inner_corners_x': 6,
                             'inner_corners_y': 5, 'square_length': 0.03})
    spec = BoardSpec('checkerboard', 7, 6, 0.03)
    assert spec.inner_corners == (6, 5)
    assert BoardSpec.from_dict(spec.to_dict()) == spec


@pytest.mark.parametrize('bad', [
    dict(CHARUCO, marker_length=0.05),            # marker larger than square
    dict(CHARUCO, dictionary='DICT_NOPE'),
    dict(CHARUCO, squares_x=20, squares_y=20, dictionary='DICT_5X5_50'),  # too few IDs
    dict(CHARUCO, kind='circles'),
])
def test_invalid_charuco_specs(bad):
    with pytest.raises((BoardSpecError, ValueError)):
        BoardSpec(**bad)


def test_checkerboard_corner_ids_and_coordinates():
    spec = BoardSpec('checkerboard', 7, 6, 0.03)
    pts = spec.corner_points
    assert pts.shape == (30, 3)
    np.testing.assert_allclose(pts[0], [0.03, 0.03, 0])
    np.testing.assert_allclose(pts[1], [0.06, 0.03, 0])      # along x first
    np.testing.assert_allclose(pts[6], [0.03, 0.06, 0])      # next row


@pytest.mark.parametrize('spec', [BoardSpec(**CHARUCO), BoardSpec('checkerboard', 7, 6, 0.03)])
def test_texture_map_matches_detected_corners(spec):
    """tex_from_board puts every corner where a detector finds it in the flat image."""
    tex, H = board_texture(spec, 60, 40)
    if spec.kind == 'checkerboard':
        ok, c = cv2.findChessboardCornersSB(tex, spec.inner_corners,
                                            flags=cv2.CALIB_CB_ACCURACY)
        assert ok
        cols, rows = spec.inner_corners
        px = canonical_checkerboard_order(c.reshape(rows, cols, 2).astype(float), tex)
        px = px.reshape(-1, 2)
        pts = spec.corner_points
    else:
        d = cv_compat.get_dictionary(spec.dictionary)
        mc, mi = cv_compat.detect_markers(tex, d)
        ids, px = cv_compat.interpolate_charuco(tex, spec.opencv_board, mc, mi)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
        px = cv2.cornerSubPix(tex, px.astype(np.float32).reshape(-1, 1, 2), (4, 4), (-1, -1),
                              crit).reshape(-1, 2)
        pts = spec.corner_points[ids]
    pred = (np.c_[pts[:, :2], np.ones(len(pts))] @ H.T)[:, :2]
    np.testing.assert_allclose(px, pred, atol=0.1)


def test_flat_charuco_corners_are_exact_after_refinement():
    """OpenCV's interpolated corners are ~0.5 px off; detection.py refines them.
    The final corners must sit on the texture's true corner positions."""
    from handeye_autopilot.core.camera import CameraModel
    from handeye_autopilot.core.config import CaptureConfig
    from handeye_autopilot.core.detection import detect
    spec = BoardSpec(**CHARUCO)
    tex, H = board_texture(spec, 60, 40)
    flat = CameraModel(np.eye(3), np.zeros(5), tex.shape[1], tex.shape[0])
    det = detect(tex, spec, flat, CaptureConfig(masking=False))
    pred = (np.c_[det.object_points[:, :2], np.ones(len(det.ids))] @ H.T)[:, :2]
    assert np.abs(det.image_points - pred).max() < 0.05


def test_board_image_cli_writes_exact_scale(tmp_path, capsys):
    out = tmp_path / 'board.png'
    # 0.0254 m at 100 dpi is exactly 100 px per square
    rc = board_image.main(['checkerboard', '7', '6', '--square', '0.0254', '--dpi', '100',
                           '--margin', '0.0254', '-o', str(out)])
    assert rc == 0
    assert board_image.read_dpi(out.read_bytes()) == pytest.approx(100, abs=0.05)
    img = cv2.imread(str(out), cv2.IMREAD_GRAYSCALE)
    assert img.shape == (6 * 100 + 200, 7 * 100 + 200)
    assert img[150, 150] == 0 and img[150, 250] == 255   # first square black


@pytest.mark.parametrize('args', [['--dpi', '0'], ['--dpi', '-5'], ['--margin', '-0.01'],
                                  ['--dpi', '5']])
def test_board_image_cli_rejects_bad_numbers(tmp_path, args):
    rc = board_image.main(['checkerboard', '7', '6', '--square', '0.03', *args, '-o',
                           str(tmp_path / 'x.png')])
    assert rc == 2


def test_board_image_cli_refuses_symmetric_board(tmp_path, capsys):
    rc = board_image.main(['checkerboard', '8', '6', '--square', '0.03', '-o',
                           str(tmp_path / 'x.png')])
    assert rc == 2
    assert 'symmetric' in capsys.readouterr().err
