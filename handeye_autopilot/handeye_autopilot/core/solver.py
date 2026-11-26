"""Hand-eye solver: initial guess, refinement, outliers, observability (02-features §E).

Reuse: OpenCV calibrateHandEye / calibrateRobotWorldHandEye (all methods, via
the mode adapter), SciPy least_squares (soft_l1), median_abs_deviation,
Rotation. Written here: the residual function, the observability measures as
defined in §E4, and the pipeline order.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations

import cv2
import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.stats import median_abs_deviation

from . import handeye
from . import transforms as tf
from .capture import CaptureRecord
from .config import SolverConfig

Mat = NDArray[np.float64]

# Solve-failure reasons; the ROS node maps them to Calibrate result codes:
# TOO_FEW_CAPTURES -> NOT_ENOUGH_SAMPLES, the others -> SOLVE_FAILED.
TOO_FEW_CAPTURES = 'too_few_captures'
DEGENERATE_ROTATION = 'degenerate_rotation'
WEAK_INFORMATION = 'weak_information'
NO_INITIAL_GUESS = 'no_initial_guess'
REFINEMENT_FAILED = 'refinement_failed'


class SolveError(RuntimeError):
    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


@dataclass
class Observability:
    rotation_amplitude: float = float('nan')       # rad, max over pairs
    axis_spread: float = float('nan')              # 2 * lambda2, 0..1
    scaled_min_singular_value: float = float('nan')  # 1/m


@dataclass
class Solution:
    X: Mat
    Z: Mat
    inliers: list[int]                 # capture_ids kept
    outliers: list[int]                # capture_ids removed
    initial_method: str
    capture_rms_px: dict[int, float]   # per capture, robot-predicted, at the solution
    reprojection_rms_px: float         # over all inlier corners
    reprojection_max_px: float
    observability: Observability = field(default_factory=Observability)


# -- residuals ------------------------------------------------------------

def capture_errors(mode: str, X: Mat, Z: Mat, rec: CaptureRecord) -> Mat:
    """(N, 2) pixel errors of the robot-predicted board pose for one capture."""
    assert rec.tool_pose is not None and rec.object_points is not None
    assert rec.image_points is not None
    C = handeye.predict_board(mode, X, Z, rec.tool_pose)
    return rec.camera.project_object(C, rec.object_points) - rec.image_points


def _apply(X0: Mat, Z0: Mat, p: Mat) -> tuple[Mat, Mat]:
    """Local tangent update: R = R0 . exp(dr), t = t0 + dt; p = [drX, dtX, drZ, dtZ]."""
    X = tf.make(X0[:3, :3] @ Rotation.from_rotvec(p[0:3]).as_matrix(), X0[:3, 3] + p[3:6])
    Z = tf.make(Z0[:3, :3] @ Rotation.from_rotvec(p[6:9]).as_matrix(), Z0[:3, 3] + p[9:12])
    return X, Z


def _residuals(mode: str, X0: Mat, Z0: Mat, recs: list[CaptureRecord], p: Mat) -> Mat:
    X, Z = _apply(X0, Z0, p)
    return np.concatenate([capture_errors(mode, X, Z, r).ravel() for r in recs])


def _rms(e: Mat) -> float:
    return float(np.sqrt(np.mean(np.sum(e.reshape(-1, 2) ** 2, axis=1))))


# -- initial guess (§E1) ----------------------------------------------------

def initial_guess(mode: str, recs: list[CaptureRecord]) -> tuple[Mat, Mat, str]:
    """Every OpenCV method through the adapter; the best robot-predicted reprojection wins."""
    G = [r.tool_pose for r in recs]
    C = [r.camera_T_board for r in recs]
    best: tuple[float, Mat, Mat, str] | None = None
    candidates = [(m, handeye.opencv_hand_eye) for m in handeye.HAND_EYE_METHODS] + \
        [(m, handeye.opencv_robot_world) for m in handeye.ROBOT_WORLD_METHODS]
    for method, fn in candidates:
        try:
            X, Z = fn(mode, G, C, method)  # type: ignore[arg-type]
        except cv2.error:
            continue
        if not (np.all(np.isfinite(X)) and np.all(np.isfinite(Z))):
            continue
        err = np.concatenate([capture_errors(mode, X, Z, r).ravel() for r in recs])
        score = _rms(err) if np.all(np.isfinite(err)) else np.inf
        if best is None or score < best[0]:
            best = (score, X, Z, method)
    if best is None or not np.isfinite(best[0]):
        raise SolveError(NO_INITIAL_GUESS, 'no OpenCV hand-eye method gave a usable estimate')
    return best[1], best[2], best[3]


# -- refinement (§E2) -------------------------------------------------------

def refine(mode: str, recs: list[CaptureRecord], X0: Mat, Z0: Mat, cfg: SolverConfig
           ) -> tuple[Mat, Mat]:
    res = least_squares(lambda p: _residuals(mode, X0, Z0, recs, p), np.zeros(12),
                        loss='soft_l1', f_scale=cfg.f_scale_sigmas * cfg.sigma_px,
                        x_scale='jac', method='trf')
    if not res.success or not np.all(np.isfinite(res.x)):
        raise SolveError(REFINEMENT_FAILED, f'refinement did not converge: {res.message}')
    return _apply(X0, Z0, res.x)


# -- observability (§E4) ----------------------------------------------------

def rotation_measures(mode: str, recs: list[CaptureRecord], cfg: SolverConfig
                      ) -> tuple[float, float]:
    """(rotation amplitude, axis spread) over all pairs of moving rotations A_i."""
    R = [handeye.moving(mode, r.tool_pose)[:3, :3] for r in recs  # type: ignore[arg-type]
         ]
    amps, axes = [], []
    for a, b in combinations(R, 2):
        rv = Rotation.from_matrix(a.T @ b).as_rotvec()
        theta = float(np.linalg.norm(rv))
        amps.append(theta)
        if theta >= cfg.min_pair_rotation:
            axes.append(rv / theta)
    amplitude = max(amps) if amps else 0.0
    if len(axes) < 2:
        return amplitude, 0.0
    A = np.array(axes)
    M = A.T @ A / len(A)
    lam = np.sort(np.linalg.eigvalsh(M))[::-1]
    return amplitude, float(np.clip(2.0 * lam[1], 0.0, 1.0))


def scaled_min_singular_value(mode: str, recs: list[CaptureRecord], X: Mat, Z: Mat,
                              cfg: SolverConfig) -> float:
    """§E4: sqrt(w) . J_r / sigma with soft_l1 weights; rotation columns / median depth."""
    def r(p: Mat) -> Mat:
        return _residuals(mode, X, Z, recs, p)
    r0 = r(np.zeros(12))
    h = 1e-6
    J = np.empty((r0.size, 12))
    for k in range(12):
        dp = np.zeros(12)
        dp[k] = h
        J[:, k] = (r(dp) - r(-dp)) / (2 * h)
    s = cfg.f_scale_sigmas * cfg.sigma_px
    w = 1.0 / np.sqrt(1.0 + (r0 / s) ** 2)
    J = J * (np.sqrt(w) / cfg.sigma_px)[:, None]
    # median optical-axis depth (z) of the predicted board origins (§E4)
    depth = float(np.median([
        handeye.predict_board(mode, X, Z, rec.tool_pose)[2, 3]  # type: ignore[arg-type]
        for rec in recs]))
    if not np.isfinite(depth) or depth <= 0:
        raise SolveError(WEAK_INFORMATION, f'board depth {depth:.3f} m is not in front of '
                         'the camera')
    for cols in (slice(0, 3), slice(6, 9)):
        J[:, cols] /= depth
    return float(np.linalg.svd(J, compute_uv=False).min())


def check_rotations(mode: str, recs: list[CaptureRecord], cfg: SolverConfig,
                    obs: Observability) -> None:
    obs.rotation_amplitude, obs.axis_spread = rotation_measures(mode, recs, cfg)
    if obs.rotation_amplitude < cfg.min_rotation_amplitude or \
            obs.axis_spread < cfg.min_axis_spread:
        raise SolveError(
            DEGENERATE_ROTATION,
            f'rotations too small or about one axis (largest '
            f'{np.degrees(obs.rotation_amplitude):.1f} deg, axis spread '
            f'{obs.axis_spread:.2f}): add rotation about another axis')


# -- full pipeline (§E) -----------------------------------------------------

def solve(mode: str, records: list[CaptureRecord], cfg: SolverConfig) -> Solution:
    """Accepted records -> (X, Z) with outlier rejection and observability checks.

    Raises SolveError with a reason instead of returning a confident wrong answer.
    """
    handeye.moving(mode, np.eye(4))           # validates the mode
    recs = [r for r in records if r.accepted]
    if len(recs) < cfg.min_captures:
        raise SolveError(TOO_FEW_CAPTURES,
                         f'{len(recs)} accepted captures < {cfg.min_captures}')
    obs = Observability()
    check_rotations(mode, recs, cfg, obs)
    X0, Z0, method = initial_guess(mode, recs)
    X, Z = refine(mode, recs, X0, Z0, cfg)

    # outlier rejection (§E3): per-capture RMS, median + k * MAD with a floor, refit once
    rms = np.array([_rms(capture_errors(mode, X, Z, r)) for r in recs])
    limit = max(float(np.median(rms) + cfg.outlier_k * median_abs_deviation(rms, scale='normal')),
                cfg.outlier_floor_px)
    keep = [r for r, e in zip(recs, rms, strict=True) if e <= limit]
    outliers = [r.capture_id for r, e in zip(recs, rms, strict=True) if e > limit]
    if outliers:
        if len(keep) < cfg.min_captures:
            raise SolveError(TOO_FEW_CAPTURES, f'{len(keep)} captures left after removing '
                             f'{len(outliers)} outliers < {cfg.min_captures}')
        check_rotations(mode, keep, cfg, obs)
        X, Z = refine(mode, keep, X, Z, cfg)

    obs.scaled_min_singular_value = scaled_min_singular_value(mode, keep, X, Z, cfg)
    if obs.scaled_min_singular_value < cfg.min_scaled_singular_value:
        raise SolveError(WEAK_INFORMATION,
                         f'scaled smallest singular value {obs.scaled_min_singular_value:.1f} '
                         f'< {cfg.min_scaled_singular_value}: add more varied poses')
    errs = {r.capture_id: capture_errors(mode, X, Z, r) for r in keep}
    allc = np.concatenate([e.reshape(-1, 2) for e in errs.values()])
    norms = np.linalg.norm(allc, axis=1)
    return Solution(X, Z, [r.capture_id for r in keep], outliers, method,
                    {k: _rms(e) for k, e in errs.items()}, _rms(allc), float(norms.max()), obs)
