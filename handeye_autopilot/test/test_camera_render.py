"""Camera model validation and renderer accuracy (01-architecture §7, §10)."""

import math

import cv2
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.boards import BoardSpec, board_texture
from handeye_autopilot.core.camera import CameraModel, CameraModelError
from handeye_autopilot.core.detection import canonical_checkerboard_order
from handeye_autopilot.core.render import (
    BACK_FACE_GRAY,
    ImageEffects,
    PlanarObject,
    Renderer,
    RobotPoseNoise,
    look_at_board,
)
import numpy as np
import pytest

K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1.0]])


@pytest.mark.parametrize('kwargs, msg', [
    (dict(K=np.zeros((3, 3)), D=np.zeros(5)), 'uncalibrated'),
    (dict(K=K, D=np.zeros(4), distortion_model='equidistant'), 'unsupported'),
    (dict(K=K, D=np.zeros(4)), 'needs 5'),
])
def test_camera_model_rejects_bad_input(kwargs, msg):
    with pytest.raises(CameraModelError, match=msg):
        CameraModel(width=640, height=480, **kwargs)


def test_project_and_normalize_round_trip():
    cam = CameraModel(K, np.array([-0.2, 0.05, 0.001, -0.001, 0.0]), 640, 480)
    pts = np.array([[0.1, -0.05, 0.6], [-0.2, 0.1, 0.8], [0.0, 0.0, 1.0]])
    px = cam.project(pts)
    np.testing.assert_allclose(cam.normalized(px), pts[:, :2] / pts[:, 2:], atol=1e-6)
    assert CameraModel.from_dict(cam.to_dict()).to_dict() == cam.to_dict()


@pytest.mark.parametrize('D', [np.zeros(5), np.array([-0.15, 0.05, 0.0005, -0.0005, 0.0])])
def test_renderer_puts_corners_where_the_camera_model_says(D):
    """Checkerboard corners found by OpenCV in a render agree with projectPoints."""
    cam = CameraModel(K, D, 640, 480)
    spec = BoardSpec('checkerboard', 7, 6, 0.03)
    tex, H = board_texture(spec, 60, 30)
    T = look_at_board(np.array([0.105, 0.09, 0]), 0.5, math.radians(20), math.radians(-25),
                      math.radians(15))
    img = Renderer(cam, ImageEffects(noise_sigma=0)).render([PlanarObject(tex, H, T)])
    ok, c = cv2.findChessboardCornersSB(img, spec.inner_corners, flags=cv2.CALIB_CB_ACCURACY)
    assert ok
    cols, rows = spec.inner_corners
    px = canonical_checkerboard_order(c.reshape(rows, cols, 2).astype(float), img)
    truth = cam.project_object(T, spec.corner_points)
    err = np.linalg.norm(px.reshape(-1, 2) - truth, axis=1)
    assert err.max() < 0.15, err.max()


def test_renderer_back_face_and_determinism():
    cam = CameraModel(np.array([[60, 0, 32], [0, 60, 24], [0, 0, 1.0]]), np.zeros(5), 64, 48)
    spec = BoardSpec('checkerboard', 7, 6, 0.03)
    tex, H = board_texture(spec, 20)
    T = look_at_board(np.array([0.105, 0.09, 0]), 0.5, math.pi, 0.0, 0.0)  # seen from behind
    r = Renderer(cam, ImageEffects(noise_sigma=0, background=255))
    img = r.render([PlanarObject(tex, H, T)])
    assert img[24, 32] == BACK_FACE_GRAY
    r2 = Renderer(cam, ImageEffects(noise_sigma=3))
    a = r2.render([PlanarObject(tex, H, T)], np.random.default_rng(1))
    b = r2.render([PlanarObject(tex, H, T)], np.random.default_rng(1))
    assert np.array_equal(a, b)


def test_robot_pose_noise_true_vs_reported():
    rng = np.random.default_rng(0)
    bias = tf.make(np.eye(3), [0.001, 0, 0])
    true_T = tf.make(np.eye(3), [0.4, 0.1, 0.3])
    reported = RobotPoseNoise(systematic=bias).report(true_T, rng)
    np.testing.assert_allclose(reported[:3, 3], [0.401, 0.1, 0.3])
    noisy = RobotPoseNoise(sigma_translation=0.001).report(true_T, rng)
    assert 0 < tf.translation_distance(noisy, true_T) < 0.01
