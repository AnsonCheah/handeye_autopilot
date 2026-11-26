"""Thresholds for detection and per-capture gates (02-features §D).

Every default is a placeholder marked *(tune)* in the design: it is set from
simulation sweeps (M1b) and real-image fixtures (M5), never copied from
elsewhere. Angles in radians, lengths in metres, image quantities in pixels.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from typing import Any


@dataclass(frozen=True)
class CaptureConfig:
    # features (§D3)
    min_corners: int = 12
    min_hull_fraction: float = 0.15        # detected corners' hull / board area
    min_collinearity: float = 0.15         # sigma2/sigma1 of corner board coords
    min_pattern_agreement: float = 0.97    # checkerboard squares matching the model
    # image quality
    min_sharpness: float = 150.0           # variance of Laplacian in the board region;
    #                                        usable real OpenCV sample images score 290-1100
    border_margin_px: float = 8.0
    # PnP fit after solvePnPRefineLM
    max_reprojection_mean_px: float = 1.0
    max_reprojection_worst_px: float = 3.0
    # pose ambiguity: rejected when e2 - e1 < k * sigma AND poses differ
    sigma_px: float = 0.3                  # pixel noise scale
    ambiguity_k: float = 2.0
    ambiguity_min_rotation: float = math.radians(4.0)
    ambiguity_min_translation_frac: float = 0.02   # of the board distance
    # stray-marker masking (§D4, ChArUco)
    masking: bool = True
    ransac_reprojection_px: float = 3.0
    min_markers: int = 4
    min_marker_hull_fraction: float = 0.15
    two_boards_min_markers: int = 4
    two_boards_ratio: float = 0.3          # second group >= ratio * inliers
    # subpixel refinement of ChArUco corners (see detection.py)
    subpix_max_half_window: int = 8
    # consensus over N frames (§D2)
    consensus_min_frames: int = 3
    consensus_max_translation_frac: float = 0.004  # of the board distance
    consensus_max_rotation: float = math.radians(0.5)
    max_frames_without_tf: int = 1       # frames dropped for missing TF at their stamp
    max_failed_frames: int = 1           # frames rejected by a gate, of the N taken

    def __post_init__(self) -> None:
        if self.min_corners < 4:
            raise ValueError('min_corners must be >= 4 (PnP needs four points)')
        if self.consensus_min_frames < 1:
            raise ValueError('consensus_min_frames must be >= 1')

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> CaptureConfig:
        known = {f.name for f in fields(cls)}
        unknown = set(d) - known
        if unknown:
            raise ValueError(f'unknown capture config keys: {sorted(unknown)}')
        return cls(**d)


@dataclass(frozen=True)
class SolverConfig:
    """Solver, observability and quality-gate settings (02-features §E-G).

    Gate limits are placeholders until the M1b simulation sweep and M5 real
    runs set them (*(tune)* in the design). SI units; reprojection in px.
    """

    # refinement (§E2)
    sigma_px: float = 0.3                  # pixel noise scale
    f_scale_sigmas: float = 3.0            # soft_l1 f_scale = this * sigma
    # outlier rejection (§E3)
    outlier_k: float = 4.0                 # median + k * MAD (normal-scaled)
    outlier_floor_px: float = 1.0
    min_captures: int = 8
    # observability (§E4)
    min_pair_rotation: float = math.radians(5.0)   # pairs used for axis spread
    min_rotation_amplitude: float = math.radians(20.0)
    min_axis_spread: float = 0.2
    min_scaled_singular_value: float = 50.0        # 1/m
    # k-fold (§G)
    folds: int = 5
    # quality gates (§F)
    max_reprojection_rms_px: float = 1.0
    max_reprojection_px: float = 4.0
    # first M1b sweep (sigma 0.2 px, robot 0.2 mm / 0.02 deg, 20 captures): worst
    # fold 1.1 mm / 0.18 deg; limits leave margin until real runs (M5) set them
    max_fold_translation: float = 0.003           # m
    max_fold_rotation: float = math.radians(0.5)

    def __post_init__(self) -> None:
        for name in ('folds', 'min_captures'):
            if not isinstance(getattr(self, name), int) or isinstance(getattr(self, name), bool):
                raise ValueError(f'{name} must be an integer')
        if self.folds < 2:
            raise ValueError('folds must be >= 2')
        if self.min_captures < 3:
            raise ValueError('min_captures must be >= 3')
        positive = ('sigma_px', 'f_scale_sigmas', 'outlier_k', 'min_pair_rotation',
                    'max_reprojection_rms_px', 'max_reprojection_px', 'max_fold_translation',
                    'max_fold_rotation')
        non_negative = ('outlier_floor_px', 'min_rotation_amplitude', 'min_axis_spread',
                        'min_scaled_singular_value')
        for name in positive + non_negative:
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f'{name} must be a finite number')
            if name in positive and v <= 0:
                raise ValueError(f'{name} must be > 0')
            if v < 0:
                raise ValueError(f'{name} must be >= 0')
        if self.min_axis_spread > 1:
            raise ValueError('min_axis_spread is a ratio in [0, 1]')
        loss_scale = self.sigma_px * self.f_scale_sigmas
        if not math.isfinite(loss_scale) or loss_scale <= 1e-9:
            raise ValueError('sigma_px * f_scale_sigmas must be a finite positive pixel scale')

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> SolverConfig:
        known = {f.name for f in fields(cls)}
        unknown = set(d) - known
        if unknown:
            raise ValueError(f'unknown solver config keys: {sorted(unknown)}')
        return cls(**d)
