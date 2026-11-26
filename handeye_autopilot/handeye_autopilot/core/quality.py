"""k-fold verification, quality gates and the run report (02-features §F-G).

Group names say what each gate measures; none of them is a confidence
interval. Shared errors (intrinsics, robot kinematics, board print scale) move
every fold the same way and pass every gate (tests record their bias).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Any

import numpy as np

from . import transforms as tf
from .capture import CaptureRecord
from .config import SolverConfig
from .solver import Observability, Solution, SolveError, capture_errors, solve

SANITY = 'sanity'
FOLD_SENSITIVITY = 'fold_sensitivity'
ACCURACY_EVIDENCE = 'accuracy_evidence'
NAN = float('nan')


@dataclass
class Gate:
    """Mirrors handeye_autopilot_interfaces/GateResult."""

    name: str
    group: str
    unit: str
    comparison: str      # '<=' or '>='
    value: float
    limit: float
    gated: bool = True

    @property
    def passed(self) -> bool:
        """Always False for measurement-only rows (comparison '')."""
        if not self.comparison or not math.isfinite(self.value):
            return False
        return self.value <= self.limit if self.comparison == '<=' else self.value >= self.limit

    def to_dict(self) -> dict[str, Any]:
        return dict(asdict(self), passed=self.passed)


@dataclass
class FoldResult:
    index: int
    held_out: list[int]                 # capture_ids
    ok: bool
    reason: str = ''
    translation: float = NAN            # m, vs all-data X
    rotation: float = NAN               # rad, vs all-data X


@dataclass
class QualityReport:
    """Mirrors handeye_autopilot_interfaces/QualityReport (NaN = not computed)."""

    gates: list[Gate] = field(default_factory=list)
    captures_accepted: int = 0
    captures_rejected: int = 0
    inliers: int = 0
    outliers: int = 0
    reprojection_rms_px: float = NAN
    reprojection_max_px: float = NAN
    relative_rotation_max_rad: float = NAN
    rotation_axis_spread: float = NAN
    scaled_min_singular_value: float = NAN
    folds: int = 0
    folds_failed: int = 0
    fold_translation_max_m: float = NAN
    fold_rotation_max_rad: float = NAN
    heldout_reprojection_rms_px: float = NAN
    heldout_reprojection_max_px: float = NAN

    @property
    def passed(self) -> bool:
        gated = [g for g in self.gates if g.gated]
        return bool(gated) and all(g.passed for g in gated)

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d['gates'] = [g.to_dict() for g in self.gates]
        d['passed'] = self.passed
        return d


@dataclass
class Calibration:
    mode: str
    solution: Solution | None
    report: QualityReport
    folds: list[FoldResult]
    failure: SolveError | None = None

    @property
    def transform_valid(self) -> bool:
        return self.solution is not None

    @property
    def passed(self) -> bool:
        return self.transform_valid and self.report.passed


def k_folds(mode: str, recs: list[CaptureRecord], full: Solution, cfg: SolverConfig
            ) -> tuple[list[FoldResult], list[float]]:
    """§G1: folds by index modulo k in acquisition order; full pipeline per fold.

    Returns the fold results and the held-out corner errors (px) pooled over
    the successful folds.
    """
    ordered = sorted(recs, key=lambda r: r.capture_id)
    results, held_errors = [], []
    if len(ordered) < cfg.folds:
        # An empty fold would train on everything and look "successful".
        return [FoldResult(k, [], ok=False, reason=f'{len(ordered)} captures < {cfg.folds} '
                           'folds') for k in range(cfg.folds)], []
    for k in range(cfg.folds):
        held = [r for i, r in enumerate(ordered) if i % cfg.folds == k]
        train = [r for i, r in enumerate(ordered) if i % cfg.folds != k]
        fold = FoldResult(k, [r.capture_id for r in held], ok=False)
        try:
            sol = solve(mode, train, cfg)
        except SolveError as e:
            fold.reason = f'{e.reason}: {e}'
            results.append(fold)
            continue
        fold.ok = True
        fold.translation = tf.translation_distance(sol.X, full.X)
        fold.rotation = tf.rotation_angle(sol.X, full.X)
        for r in held:
            held_errors.extend(np.linalg.norm(capture_errors(mode, sol.X, sol.Z, r), axis=1))
        results.append(fold)
    return results, held_errors


def calibrate(mode: str, records: list[CaptureRecord], cfg: SolverConfig) -> Calibration:
    """Solve, verify and gate one run. Never raises SolveError: failures are reported."""
    accepted = [r for r in records if r.accepted]
    report = QualityReport(captures_accepted=len(accepted),
                           captures_rejected=len(records) - len(accepted))
    try:
        sol = solve(mode, accepted, cfg)
    except SolveError as e:
        return Calibration(mode, None, report, [], e)

    obs: Observability = sol.observability
    report.inliers, report.outliers = len(sol.inliers), len(sol.outliers)
    report.reprojection_rms_px = sol.reprojection_rms_px
    report.reprojection_max_px = sol.reprojection_max_px
    report.relative_rotation_max_rad = obs.rotation_amplitude
    report.rotation_axis_spread = obs.axis_spread
    report.scaled_min_singular_value = obs.scaled_min_singular_value

    # Folds over ALL accepted captures: each training split runs its own outlier
    # rejection, so nothing decided with the held-out captures leaks in (§G1).
    folds, held = k_folds(mode, accepted, sol, cfg)
    good = [f for f in folds if f.ok]
    report.folds, report.folds_failed = len(folds), len(folds) - len(good)
    if good:
        report.fold_translation_max_m = max(f.translation for f in good)
        report.fold_rotation_max_rad = max(f.rotation for f in good)
    if held:
        h = np.asarray(held)
        report.heldout_reprojection_rms_px = float(np.sqrt(np.mean(h ** 2)))
        report.heldout_reprojection_max_px = float(h.max())

    fold_t = report.fold_translation_max_m if not report.folds_failed else NAN
    fold_r = report.fold_rotation_max_rad if not report.folds_failed else NAN
    report.gates = [
        Gate('min_inliers', SANITY, 'count', '>=', float(report.inliers), cfg.min_captures),
        Gate('reprojection_rms', SANITY, 'px', '<=', report.reprojection_rms_px,
             cfg.max_reprojection_rms_px),
        Gate('reprojection_max', SANITY, 'px', '<=', report.reprojection_max_px,
             cfg.max_reprojection_px),
        Gate('rotation_amplitude', SANITY, 'rad', '>=', obs.rotation_amplitude,
             cfg.min_rotation_amplitude),
        Gate('rotation_axis_spread', SANITY, 'ratio', '>=', obs.axis_spread,
             cfg.min_axis_spread),
        Gate('scaled_min_singular_value', SANITY, '1/m', '>=', obs.scaled_min_singular_value,
             cfg.min_scaled_singular_value),
        Gate('folds_failed', FOLD_SENSITIVITY, 'count', '<=', float(report.folds_failed), 0.0),
        Gate('fold_translation_max', FOLD_SENSITIVITY, 'm', '<=', fold_t,
             cfg.max_fold_translation),
        Gate('fold_rotation_max', FOLD_SENSITIVITY, 'rad', '<=', fold_r,
             cfg.max_fold_rotation),
        # accuracy evidence: measurements only (no limit, never gates the run)
        Gate('heldout_reprojection_rms', ACCURACY_EVIDENCE, 'px', '',
             report.heldout_reprojection_rms_px, NAN, gated=False),
        Gate('heldout_reprojection_max', ACCURACY_EVIDENCE, 'px', '',
             report.heldout_reprojection_max_px, NAN, gated=False),
    ]
    return Calibration(mode, sol, report, folds)
