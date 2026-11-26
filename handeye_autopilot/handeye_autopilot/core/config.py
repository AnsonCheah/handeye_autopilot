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
    min_sharpness: float = 400.0           # variance of Laplacian in the board region
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
