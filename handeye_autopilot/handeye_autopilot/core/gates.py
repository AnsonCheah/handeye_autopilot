"""Per-capture gates, each with its reason code (02-features §D3).

Order: features and image quality (no pose needed) -> PnP fit -> reprojection,
board outline inside the image, pose ambiguity. The first failing gate names
the rejection; computed metrics are returned either way. A PnP failure is a
REPROJECTION rejection, never an exception.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from numpy.typing import NDArray

from . import codes
from . import transforms as tf
from .boards import BoardSpec
from .camera import CameraModel
from .config import CaptureConfig
from .detection import Detection, hull_fraction
from .pose import BoardPose, PoseError, solve

OUTLINE_SAMPLES_PER_EDGE = 50


@dataclass
class GateOutcome:
    reject: str | None
    detail: str = ''
    metrics: dict[str, float] = field(default_factory=dict)

    @property
    def accepted(self) -> bool:
        return self.reject is None


def evaluate(det: Detection, gray: NDArray[np.uint8], spec: BoardSpec, camera: CameraModel,
             cfg: CaptureConfig) -> tuple[GateOutcome, BoardPose | None]:
    """All gates for one frame. Returns the outcome and the pose (if it was solved)."""
    out = prefit(det, gray, spec, cfg)
    if not out.accepted:
        return out, None
    try:
        pose = solve(det.object_points, det.image_points, camera)
    except PoseError as e:
        out.reject, out.detail = codes.REPROJECTION, f'no board pose: {e}'
        return out, None
    return postfit(pose, spec, camera, cfg, out.metrics), pose


def prefit(det: Detection, gray: NDArray[np.uint8], spec: BoardSpec,
           cfg: CaptureConfig) -> GateOutcome:
    m: dict[str, float] = dict(det.info)
    if not det.ok:
        return GateOutcome(det.reject, det.detail, m)
    n = len(det.ids)
    m['corners'] = float(n)
    if n < cfg.min_corners:
        return GateOutcome(codes.FEW_FEATURES, f'{n} corners < {cfg.min_corners}', m)
    m['collinearity'] = collinearity(det.object_points)
    if m['collinearity'] < cfg.min_collinearity:
        return GateOutcome(codes.COLLINEAR, f'corners nearly collinear '
                           f'({m["collinearity"]:.3f} < {cfg.min_collinearity})', m)
    m['hull_fraction'] = hull_fraction(det.object_points, spec)
    if m['hull_fraction'] < cfg.min_hull_fraction:
        return GateOutcome(codes.POOR_SPREAD, f'corners cover {m["hull_fraction"]:.2f} of the '
                           f'board < {cfg.min_hull_fraction}', m)
    m['sharpness'] = sharpness(gray, det.image_points)
    if m['sharpness'] < cfg.min_sharpness:
        return GateOutcome(codes.BLURRY, f'sharpness {m["sharpness"]:.1f} < '
                           f'{cfg.min_sharpness}', m)
    return GateOutcome(None, '', m)


def postfit(pose: BoardPose, spec: BoardSpec, camera: CameraModel, cfg: CaptureConfig,
            metrics: dict[str, float]) -> GateOutcome:
    m = dict(metrics)
    best = pose.best
    m.update(reprojection_mean_px=best.mean_px, reprojection_worst_px=best.worst_px,
             reprojection_rms_px=best.rms_px)
    if best.mean_px > cfg.max_reprojection_mean_px or \
            best.worst_px > cfg.max_reprojection_worst_px:
        return GateOutcome(codes.REPROJECTION, f'reprojection mean {best.mean_px:.2f} px, '
                           f'worst {best.worst_px:.2f} px', m)

    margin = outline_margin(best.camera_T_board, spec, camera)
    m['border_margin_px'] = margin
    if margin < cfg.border_margin_px:
        return GateOutcome(codes.NEAR_BORDER, f'board outline {margin:.1f} px from the image '
                           f'edge < {cfg.border_margin_px}; keep the whole board in view', m)

    amb = ambiguity(pose, cfg)
    m.update(amb)
    if amb['ambiguous'] > 0:
        return GateOutcome(codes.AMBIGUOUS_POSE,
                           f'two planar poses fit equally well (errors differ by '
                           f'{amb["error_gap_px"]:.3f} px) but differ by '
                           f'{np.degrees(amb["pose_rotation_gap"]):.1f} deg / '
                           f'{amb["pose_translation_gap"] * 1000:.0f} mm; tilt the board '
                           'more or move closer', m)
    return GateOutcome(None, '', m)


def outline_margin(camera_T_board: NDArray[np.float64], spec: BoardSpec,
                   camera: CameraModel) -> float:
    """Smallest distance (px) from the projected board outline to the image edge.

    The outline is sampled densely along its edges and projected with
    distortion; negative when it leaves the image or goes behind the camera.
    """
    corners = spec.outline
    t = np.linspace(0.0, 1.0, OUTLINE_SAMPLES_PER_EDGE, endpoint=False)[:, None]
    ends = np.roll(corners, -1, axis=0)
    pts = np.concatenate([a + t * (b - a) for a, b in zip(corners, ends, strict=True)])
    if np.any(tf.apply(camera_T_board, pts)[:, 2] <= 0):
        return -1.0
    px = camera.project_object(camera_T_board, pts)
    return float(min(px[:, 0].min(), px[:, 1].min(), camera.width - 1 - px[:, 0].max(),
                     camera.height - 1 - px[:, 1].max()))


def collinearity(object_points: NDArray[np.float64]) -> float:
    """sigma2 / sigma1 of the centred 2D board coordinates (0 = on a line)."""
    xy = object_points[:, :2] - object_points[:, :2].mean(axis=0)
    s = np.linalg.svd(xy, compute_uv=False)
    return float(s[1] / s[0]) if s[0] > 0 else 0.0


def sharpness(gray: NDArray[np.uint8], px: NDArray[np.float64]) -> float:
    """Variance of the Laplacian inside the convex hull of the detected corners."""
    hull = cv2.convexHull(px.astype(np.float32)).astype(np.int32)
    mask = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(mask, hull, 1)
    lap = cv2.Laplacian(gray, cv2.CV_64F)
    vals = lap[mask.astype(bool)]
    return float(vals.var()) if vals.size else 0.0


def ambiguity(pose: BoardPose, cfg: CaptureConfig) -> dict[str, float]:
    """IPPE ambiguity (§D3), on the two RAW IPPE hypotheses' RMS errors.

    Ambiguous when the runner-up's error is within k * sigma of the best AND
    the two poses differ (rotation or translation) by more than the limits.
    """
    if len(pose.hypotheses) < 2:
        return {'ambiguous': 0.0, 'error_gap_px': float('inf'), 'pose_rotation_gap': 0.0,
                'pose_translation_gap': 0.0}
    a, b = pose.hypotheses[0], pose.hypotheses[1]
    gap = b.rms_px - a.rms_px
    rot = tf.rotation_angle(a.camera_T_board, b.camera_T_board)
    trans = tf.translation_distance(a.camera_T_board, b.camera_T_board)
    dist = float(np.linalg.norm(a.camera_T_board[:3, 3]))
    close_errors = gap < cfg.ambiguity_k * cfg.sigma_px
    different = rot > cfg.ambiguity_min_rotation or \
        trans > cfg.ambiguity_min_translation_frac * dist
    return {'ambiguous': float(close_errors and different), 'error_gap_px': gap,
            'pose_rotation_gap': rot, 'pose_translation_gap': trans}
