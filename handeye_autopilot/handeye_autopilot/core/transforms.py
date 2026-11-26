"""4x4 homogeneous transforms (NumPy + SciPy Rotation; nothing re-implemented)."""

from __future__ import annotations

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

Mat = NDArray[np.float64]


def make(rotation: Mat | Rotation, translation: Mat) -> Mat:
    """Build ``T`` from a 3x3 matrix (or Rotation) and a translation."""
    T = np.eye(4)
    R = rotation.as_matrix() if isinstance(rotation, Rotation) else np.asarray(rotation)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return T


def inv(T: Mat) -> Mat:
    """Inverse of a rigid transform."""
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def from_rvec_tvec(rvec: Mat, tvec: Mat) -> Mat:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return make(R, tvec)


def to_rvec_tvec(T: Mat) -> tuple[Mat, Mat]:
    rvec, _ = cv2.Rodrigues(np.ascontiguousarray(T[:3, :3]))
    return rvec.reshape(3), T[:3, 3].copy()


def rotation_angle(T_a: Mat, T_b: Mat) -> float:
    """SO(3) angle between the rotations of two transforms (rad)."""
    R = T_a[:3, :3].T @ T_b[:3, :3]
    return float(Rotation.from_matrix(R).magnitude())


def translation_distance(T_a: Mat, T_b: Mat) -> float:
    return float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))


def apply(T: Mat, points: Mat) -> Mat:
    """Transform (N, 3) points."""
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    return np.asarray(P @ T[:3, :3].T + T[:3, 3], dtype=np.float64)
