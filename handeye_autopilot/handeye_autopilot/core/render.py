"""Deterministic homography renderer for tests and the synthetic camera (01-architecture §10).

Each planar object (board, stray marker, occluder) is a texture on a plane at a
true pose. For every output pixel the renderer undistorts the pixel to a ray
(``cv2.undistortPoints``), intersects the ray with each plane, keeps the
nearest hit and samples the texture with ``cv2.remap``. Lens distortion is
therefore exact up to OpenCV's undistortion accuracy.

Limits (stated, not hidden): no lens shading, no 3D occlusion beyond planar
z-ordering, no motion blur, no rolling shutter.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from numpy.typing import NDArray

from .camera import CameraModel

BACK_FACE_GRAY = 90


@dataclass
class PlanarObject:
    texture: NDArray[np.uint8]                # grayscale
    tex_from_plane: NDArray[np.float64]       # 3x3: plane (X, Y, 1) [m] -> texture px
    camera_T_plane: NDArray[np.float64]       # 4x4 true pose of the plane in the camera


@dataclass
class ImageEffects:
    noise_sigma: float = 2.0       # gray levels
    blur_sigma: float = 0.0        # px; 0 = none
    contrast: float = 1.0
    brightness: float = 0.0        # gray levels added
    background: int = 160
    supersample: int = 2           # anti-aliasing factor


@dataclass
class Renderer:
    camera: CameraModel
    effects: ImageEffects = field(default_factory=ImageEffects)

    def __post_init__(self) -> None:
        ss = self.effects.supersample
        w, h = self.camera.width * ss, self.camera.height * ss
        u, v = np.meshgrid((np.arange(w) + 0.5) / ss - 0.5, (np.arange(h) + 0.5) / ss - 0.5)
        pix = np.stack([u.ravel(), v.ravel()], axis=1)
        xy = self.camera.normalized(pix)
        self._rays = np.concatenate([xy, np.ones((xy.shape[0], 1))], axis=1)  # (N, 3)
        self._shape = (h, w)

    def render(self, objects: list[PlanarObject],
               rng: np.random.Generator | None = None) -> NDArray[np.uint8]:
        h, w = self._shape
        image = np.full((h, w), float(self.effects.background), dtype=np.float32)
        depth = np.full(h * w, np.inf)
        for obj in objects:
            R, t = obj.camera_T_plane[:3, :3], obj.camera_T_plane[:3, 3]
            n = R[:, 2]
            denom = self._rays @ n
            with np.errstate(divide='ignore', invalid='ignore'):
                s = (n @ t) / denom                      # ray length to the plane (z = s)
            hit = self._rays * s[:, None]
            plane_xy = (hit - t) @ R                     # columns: X, Y, Z(~0)
            homog = np.concatenate([plane_xy[:, :2], np.ones((h * w, 1))], axis=1)
            tex = homog @ obj.tex_from_plane.T
            tu, tv = tex[:, 0] / tex[:, 2], tex[:, 1] / tex[:, 2]
            th, tw = obj.texture.shape[:2]
            inside = (tu >= -0.5) & (tu <= tw - 0.5) & (tv >= -0.5) & (tv <= th - 0.5)
            visible = np.isfinite(s) & (s > 0) & inside & (s < depth)
            if not visible.any():
                continue
            front = denom > 0    # the ray travels along +z of the plane: printed side seen
            sampled = cv2.remap(
                obj.texture.astype(np.float32),
                tu.reshape(h, w).astype(np.float32), tv.reshape(h, w).astype(np.float32),
                interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE,
            ).ravel()
            values = np.where(front, sampled, BACK_FACE_GRAY)
            flat = image.ravel()
            flat[visible] = values[visible]
            depth[visible] = s[visible]
        return self._finish(image, rng)

    def _finish(self, image: NDArray[np.float32],
                rng: np.random.Generator | None) -> NDArray[np.uint8]:
        fx = self.effects
        ss = fx.supersample
        if ss > 1:
            image = cv2.resize(image, (self.camera.width, self.camera.height),
                               interpolation=cv2.INTER_AREA)
        if fx.blur_sigma > 0:
            image = cv2.GaussianBlur(image, (0, 0), fx.blur_sigma)
        image = image * fx.contrast + fx.brightness
        if fx.noise_sigma > 0:
            rng = rng or np.random.default_rng(0)
            image = image + rng.normal(0.0, fx.noise_sigma, image.shape)
        return np.clip(np.rint(image), 0, 255).astype(np.uint8)


def look_at_board(board_center: NDArray[np.float64], distance: float, tilt_x: float,
                  tilt_y: float, roll: float, offset_px: tuple[float, float] = (0.0, 0.0),
                  camera: CameraModel | None = None) -> NDArray[np.float64]:
    """camera_T_board for a board whose centre lies on a ray, tilted and rolled.

    ``board_center`` is the board-frame point placed on the ray (usually the
    grid centre). Angles are radians; the pivot is the board centre
    (02-features §C1). ``offset_px`` moves the centre off the principal point.
    """
    from scipy.spatial.transform import Rotation

    from . import transforms as tf
    if camera is not None and any(offset_px):
        cx, cy = camera.K[0, 2] + offset_px[0], camera.K[1, 2] + offset_px[1]
        ray = np.append(camera.normalized(np.array([[cx, cy]]))[0], 1.0)
    else:
        ray = np.array([0.0, 0.0, 1.0])
    center_cam = ray / np.linalg.norm(ray) * distance
    R = Rotation.from_euler('z', roll) * Rotation.from_euler('xy', [tilt_x, tilt_y])
    return tf.make(R, center_cam - R.apply(board_center))


@dataclass
class RobotPoseNoise:
    """True vs reported robot pose (02-features §H): reported = true · error."""

    sigma_translation: float = 0.0     # m, random per capture
    sigma_rotation: float = 0.0        # rad, random per capture
    systematic: NDArray[np.float64] = field(default_factory=lambda: np.eye(4))

    def report(self, true_T: NDArray[np.float64], rng: np.random.Generator
               ) -> NDArray[np.float64]:
        from scipy.spatial.transform import Rotation

        from . import transforms as tf
        noise = tf.make(Rotation.from_rotvec(rng.normal(0, self.sigma_rotation, 3)),
                        rng.normal(0, self.sigma_translation, 3))
        return true_T @ self.systematic @ noise
