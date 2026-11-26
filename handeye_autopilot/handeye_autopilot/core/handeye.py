"""Mode adapter: the only mode-specific code (02-features §A).

Notation: G = base_T_tool (robot), C = camera_T_board (PnP),
X = the published unknown, Z = the other unknown.

| mode        | X             | Z             | A (moving) |
| eye_to_hand | base_T_camera | tool_T_board  | inv(G)     |
| eye_in_hand | tool_T_camera | base_T_board  | G          |

Model: A . X = Z . inv(C); predicted board pose C_hat = inv(X) . inv(A) . Z.

OpenCV names "a2b" the pose of a in b (= b_T_a). Mappings, verified against an
independent oracle in test_handeye_adapter.py:
- calibrateHandEye(gripper2base, target2cam) -> cam2gripper:
  gripper2base = A (G for eye_in_hand, inv(G) for eye_to_hand), target2cam = C,
  cam2gripper = X. Z is then the rotation-averaged A . X . C.
- calibrateRobotWorldHandEye(world2cam, base2gripper) -> base2world, gripper2cam:
  world2cam = C, base2gripper = inv(A), base2world = inv(Z), gripper2cam = inv(X).
"""

from __future__ import annotations

from collections.abc import Sequence

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from . import transforms as tf

Mat = NDArray[np.float64]
MODES = ('eye_to_hand', 'eye_in_hand')

HAND_EYE_METHODS = ('TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS')
ROBOT_WORLD_METHODS = ('SHAH', 'LI')


def _check(mode: str) -> None:
    if mode not in MODES:
        raise ValueError(f'mode must be one of {MODES}, got {mode!r}')


def moving(mode: str, G: Mat) -> Mat:
    """A_i for one robot pose."""
    _check(mode)
    return tf.inv(G) if mode == 'eye_to_hand' else np.asarray(G, dtype=np.float64)


def predict_board(mode: str, X: Mat, Z: Mat, G: Mat) -> Mat:
    """Robot-predicted camera_T_board."""
    return tf.inv(X) @ tf.inv(moving(mode, G)) @ Z


def tool_pose_for(mode: str, X: Mat, Z: Mat, camera_T_board: Mat) -> Mat:
    """base_T_tool that puts the board at ``camera_T_board`` (for pose planning)."""
    A = Z @ tf.inv(camera_T_board) @ tf.inv(X)
    return tf.inv(A) if mode == 'eye_to_hand' else A


def average(transforms: Sequence[Mat]) -> Mat:
    """Rotation mean (SciPy) and translation mean."""
    R = Rotation.from_matrix(np.array([T[:3, :3] for T in transforms])).mean()
    return tf.make(R, np.mean([T[:3, 3] for T in transforms], axis=0))


def _split(Ts: Sequence[Mat]) -> tuple[list[Mat], list[Mat]]:
    return ([np.ascontiguousarray(T[:3, :3]) for T in Ts],
            [np.ascontiguousarray(T[:3, 3]).reshape(3, 1) for T in Ts])


def opencv_hand_eye(mode: str, G: Sequence[Mat], C: Sequence[Mat], method: str
                    ) -> tuple[Mat, Mat]:
    """(X, Z) from cv2.calibrateHandEye with the given method name."""
    A = [moving(mode, g) for g in G]
    Ra, ta = _split(A)
    Rc, tc = _split(C)
    flag = getattr(cv2, f'CALIB_HAND_EYE_{method}')
    R, t = cv2.calibrateHandEye(Ra, ta, Rc, tc, method=flag)
    X = tf.make(np.asarray(R, dtype=np.float64), np.asarray(t, dtype=np.float64))
    Z = average([a @ X @ c for a, c in zip(A, C, strict=True)])
    return X, Z


def opencv_robot_world(mode: str, G: Sequence[Mat], C: Sequence[Mat], method: str
                       ) -> tuple[Mat, Mat]:
    """(X, Z) from cv2.calibrateRobotWorldHandEye with the given method name."""
    Rc, tc = _split(C)
    Rb, tb = _split([tf.inv(moving(mode, g)) for g in G])
    flag = getattr(cv2, f'CALIB_ROBOT_WORLD_HAND_EYE_{method}')
    Rbw, tbw, Rgc, tgc = cv2.calibrateRobotWorldHandEye(Rc, tc, Rb, tb, method=flag)
    base2world = tf.make(np.asarray(Rbw, dtype=np.float64), np.asarray(tbw, dtype=np.float64))
    gripper2cam = tf.make(np.asarray(Rgc, dtype=np.float64), np.asarray(tgc, dtype=np.float64))
    return tf.inv(gripper2cam), tf.inv(base2world)
