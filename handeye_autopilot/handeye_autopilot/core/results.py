"""Result YAML (01-architecture §9): both transforms, quality report, config, time.

SI units and ROS quaternion order (x, y, z, w); a `human` block repeats the
transforms in mm and degrees for reports.
"""

from __future__ import annotations

from datetime import UTC, datetime
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from .config import SolverConfig
from .quality import Calibration

RESULT_SCHEMA_VERSION = 1


def transform_dict(T: NDArray[np.float64], parent: str, child: str) -> dict[str, Any]:
    q = Rotation.from_matrix(T[:3, :3]).as_quat()          # x, y, z, w
    return {'parent': parent, 'child': child,
            'translation': [float(v) for v in T[:3, 3]],
            'rotation_xyzw': [float(v) for v in q]}


def human(T: NDArray[np.float64]) -> dict[str, Any]:
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return {'translation_mm': [round(float(v) * 1000, 3) for v in T[:3, 3]],
            'rpy_deg': [round(float(v), 4) for v in rpy]}


def _clean(v: Any) -> Any:
    if isinstance(v, float) and not math.isfinite(v):
        return None                       # YAML null = not computed
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_clean(x) for x in v]
    return v


def result_dict(cal: Calibration, cfg: SolverConfig, frames: dict[str, str],
                metadata: dict[str, Any] | None = None) -> dict[str, Any]:
    """frames: base_frame, tool_frame, camera_frame, board_frame."""
    mode = cal.mode
    parent = frames['base_frame'] if mode == 'eye_to_hand' else frames['tool_frame']
    z_parent = frames['tool_frame'] if mode == 'eye_to_hand' else frames['base_frame']
    out: dict[str, Any] = {
        'schema_version': RESULT_SCHEMA_VERSION,
        'created': datetime.now(UTC).isoformat(timespec='seconds'),
        'mode': mode,
        'transform_valid': cal.transform_valid,
        'passed': cal.passed,
        'failure': None if cal.failure is None else {'reason': cal.failure.reason,
                                                     'message': str(cal.failure)},
        'quality': cal.report.to_dict(),
        'folds': [{'index': f.index, 'ok': f.ok, 'reason': f.reason,
                   'held_out': f.held_out, 'translation_m': f.translation,
                   'rotation_rad': f.rotation} for f in cal.folds],
        'solver_config': cfg.to_dict(),
        'metadata': dict(metadata or {}),
    }
    if cal.solution is not None:
        s = cal.solution
        out['camera_transform'] = transform_dict(s.X, parent, frames['camera_frame'])
        out['target_transform'] = transform_dict(s.Z, z_parent, frames.get('board_frame',
                                                                            'board'))
        out['human'] = {'camera_transform': human(s.X), 'target_transform': human(s.Z)}
        out['solution'] = {'initial_method': s.initial_method, 'inliers': s.inliers,
                           'outliers': s.outliers}
    cleaned: dict[str, Any] = _clean(out)
    return cleaned
