"""Pinhole camera model with distortion, ROS-free (01-architecture §7).

shortcut: the ROS layer builds this from CameraInfo (via image_geometry); the
core keeps K, D and image size as NumPy and uses OpenCV for projection.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray

from . import transforms as tf

# CameraInfo distortion model -> number of coefficients OpenCV expects.
SUPPORTED_MODELS = {'plumb_bob': 5, 'rational_polynomial': 8}


class CameraModelError(ValueError):
    pass


@dataclass(frozen=True)
class CameraModel:
    K: NDArray[np.float64]
    D: NDArray[np.float64]
    width: int
    height: int
    distortion_model: str = 'plumb_bob'
    rectified: bool = False

    def __post_init__(self) -> None:
        K = np.asarray(self.K, dtype=float).reshape(3, 3)
        D = np.asarray(self.D, dtype=float).reshape(-1)
        if not np.any(K):
            raise CameraModelError('camera matrix K is all zero (uncalibrated camera)')
        if K[0, 0] <= 0 or K[1, 1] <= 0:
            raise CameraModelError('focal lengths must be positive')
        if self.distortion_model not in SUPPORTED_MODELS:
            raise CameraModelError(
                f'distortion model {self.distortion_model!r} unsupported; '
                f'use one of {sorted(SUPPORTED_MODELS)}')
        n = SUPPORTED_MODELS[self.distortion_model]
        if D.size not in (0, n):
            raise CameraModelError(
                f'{self.distortion_model} needs {n} coefficients, got {D.size}')
        if D.size == 0:
            D = np.zeros(n)
        if self.width <= 0 or self.height <= 0:
            raise CameraModelError('image size must be positive')
        object.__setattr__(self, 'K', K)
        object.__setattr__(self, 'D', D)

    @property
    def size(self) -> tuple[int, int]:
        return (self.width, self.height)

    def project(self, points_cam: NDArray[np.float64]) -> NDArray[np.float64]:
        """Project (N, 3) points in the optical frame to (N, 2) distorted pixels."""
        P = np.asarray(points_cam, dtype=float).reshape(-1, 1, 3)
        px, _ = cv2.projectPoints(P, np.zeros(3), np.zeros(3), self.K, self.D)
        return np.asarray(px, dtype=np.float64).reshape(-1, 2)

    def project_object(self, camera_T_object: NDArray[np.float64],
                       points: NDArray[np.float64]) -> NDArray[np.float64]:
        rvec, tvec = tf.to_rvec_tvec(camera_T_object)
        P = np.asarray(points, dtype=float).reshape(-1, 1, 3)
        px, _ = cv2.projectPoints(P, rvec, tvec, self.K, self.D)
        return np.asarray(px, dtype=np.float64).reshape(-1, 2)

    def normalized(self, pixels: NDArray[np.float64]) -> NDArray[np.float64]:
        """Undistort (N, 2) pixels to normalized image coordinates (x/z, y/z)."""
        px = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
        out = cv2.undistortPoints(px, self.K, self.D)
        return np.asarray(out, dtype=np.float64).reshape(-1, 2)

    def to_dict(self) -> dict[str, object]:
        return {
            'K': self.K.tolist(), 'D': self.D.tolist(), 'width': self.width,
            'height': self.height, 'distortion_model': self.distortion_model,
            'rectified': self.rectified,
        }

    @classmethod
    def from_dict(cls, d: dict[str, object]) -> CameraModel:
        return cls(
            K=np.asarray(d['K'], dtype=float), D=np.asarray(d['D'], dtype=float),
            width=int(d['width']), height=int(d['height']),  # type: ignore[call-overload]
            distortion_model=str(d.get('distortion_model', 'plumb_bob')),
            rectified=bool(d.get('rectified', False)),
        )
