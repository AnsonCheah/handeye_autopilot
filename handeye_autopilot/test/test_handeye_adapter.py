"""Mode adapter vs an independent ground-truth oracle (02-features §A).

Written before the adapter, as the design asks for the OpenCV mappings. The
oracle builds tool and board poses from the physical chain only; it never
calls the adapter.
"""

from handeye_autopilot.core import handeye
from handeye_autopilot.core import transforms as tf
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

MODES = ['eye_to_hand', 'eye_in_hand']


def rand_T(rng, t_scale=0.3):
    return tf.make(Rotation.random(random_state=rng), rng.uniform(-t_scale, t_scale, 3))


def oracle(mode, n=12, seed=0):
    """Physical chain only.

    eye_to_hand: camera fixed in base (base_T_camera), board on tool (tool_T_board):
        camera_T_board = inv(base_T_camera) . base_T_tool . tool_T_board
    eye_in_hand: camera on tool (tool_T_camera), board fixed (base_T_board):
        camera_T_board = inv(tool_T_camera) . inv(base_T_tool) . base_T_board
    """
    rng = np.random.default_rng(seed)
    X, Z = rand_T(rng), rand_T(rng)
    G, C = [], []
    R0 = Rotation.random(random_state=rng)
    for _ in range(n):
        # Realistic robot poses: within ~35 deg of a nominal orientation. Tsai,
        # Horaud and Daniilidis break down for relative rotations near 180 deg.
        R = R0 * Rotation.from_rotvec(rng.uniform(-0.6, 0.6, 3))
        g = tf.make(R, rng.uniform(-0.5, 0.5, 3))
        if mode == 'eye_to_hand':
            c = tf.inv(X) @ g @ Z
        else:
            c = tf.inv(X) @ tf.inv(g) @ Z
        G.append(g)
        C.append(c)
    return X, Z, G, C


def close(A, B, t_tol=1e-6, r_tol=1e-6):
    return tf.translation_distance(A, B) < t_tol and tf.rotation_angle(A, B) < r_tol


@pytest.mark.parametrize('mode', MODES)
def test_predicted_board_pose_matches_the_physical_chain(mode):
    X, Z, G, C = oracle(mode)
    for g, c in zip(G, C, strict=True):
        assert close(handeye.predict_board(mode, X, Z, g), c)


@pytest.mark.parametrize('mode', MODES)
def test_model_equation_holds(mode):
    X, Z, G, C = oracle(mode)
    for g, c in zip(G, C, strict=True):
        A = handeye.moving(mode, g)
        assert np.allclose(A @ X, Z @ tf.inv(c), atol=1e-9)


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('method', handeye.HAND_EYE_METHODS)
def test_calibrate_hand_eye_mapping(mode, method):
    X, Z, G, C = oracle(mode, n=15)
    Xe, Ze = handeye.opencv_hand_eye(mode, G, C, method)
    assert close(Xe, X, 1e-5, 1e-5), method
    assert close(Ze, Z, 1e-5, 1e-5), method


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('method', handeye.ROBOT_WORLD_METHODS)
def test_robot_world_mapping(mode, method):
    X, Z, G, C = oracle(mode, n=15)
    Xe, Ze = handeye.opencv_robot_world(mode, G, C, method)
    assert close(Xe, X, 1e-5, 1e-5), method
    assert close(Ze, Z, 1e-5, 1e-5), method


@pytest.mark.parametrize('mode', MODES)
def test_a_swapped_inverse_fails(mode):
    """Using the other mode's A_i (i.e. inverting G) must not recover the truth."""
    X, Z, G, C = oracle(mode, n=15)
    other = 'eye_in_hand' if mode == 'eye_to_hand' else 'eye_to_hand'
    Xe, _ = handeye.opencv_hand_eye(other, G, C, handeye.HAND_EYE_METHODS[0])
    assert not close(Xe, X, 1e-3, 1e-3)
    for g, c in zip(G[:3], C[:3], strict=True):
        assert not close(handeye.predict_board(other, X, Z, g), c, 1e-3, 1e-3)


@pytest.mark.parametrize('mode', MODES)
def test_tool_pose_for_a_desired_board_pose(mode):
    """The adapter maps a camera-relative board pose back to the tool pose."""
    X, Z, G, C = oracle(mode)
    for g, c in zip(G, C, strict=True):
        assert close(handeye.tool_pose_for(mode, X, Z, c), g)


def test_unknown_mode_rejected():
    with pytest.raises(ValueError, match='mode'):
        handeye.moving('sideways', np.eye(4))
