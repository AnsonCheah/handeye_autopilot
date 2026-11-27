"""Mount sweep, core level: very different board / camera mounts are recovered
(spec/mount_sweep.yaml). A full session per case through m2_sim: bootstrap,
planning, captures, solve, gates."""

import math
import os
from pathlib import Path

from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.config import SessionConfig
from m1b_sim import truth
from m2_sim import SPEC as M2
from m2_sim import run
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

SPEC = yaml.safe_load((Path(__file__).parent / 'spec' / 'mount_sweep.yaml').read_text())
FULL = os.environ.get('HANDEYE_FULL_SWEEP') == '1'
CASES = [c for c in SPEC['cases'] if FULL or c.get('ci')]
TOL = M2['tolerance']


def mounts(mode, case):
    """(X, Z) with the robot-side mount moved by the case's offset (tool frame)."""
    X, Z = truth(mode)
    M = Z if mode == 'eye_to_hand' else X
    R = Rotation.from_euler('xyz', case.get('rotation_deg', [0, 0, 0]), degrees=True).as_matrix()
    M = tf.make(R @ M[:3, :3], M[:3, 3] + np.asarray(case.get('translation_m', [0, 0, 0])))
    return (X, M) if mode == 'eye_to_hand' else (M, Z)


@pytest.mark.parametrize('mode', ['eye_to_hand', 'eye_in_hand'])
@pytest.mark.parametrize('case', CASES, ids=[c['name'] for c in CASES])
def test_mount_is_recovered(mode, case):
    X, Z = mounts(mode, case)
    lever = (Z if mode == 'eye_to_hand' else X)[:3, 3]
    cfg = SessionConfig(**{**SessionConfig().to_dict(), **M2['session'],
                           'lever_max': float(np.abs(lever).max()) + SPEC['lever_margin_m']})
    r, Xt, _ = run(mode, mounts=(X, Z), session_cfg=cfg)
    assert r.failure is None, r.message
    cal = r.calibration
    assert cal.passed, [g.name for g in cal.report.gates if g.gated and not g.passed]
    dt = tf.translation_distance(cal.solution.X, Xt)
    dr = math.degrees(tf.rotation_angle(cal.solution.X, Xt))
    print(f'{mode} {case["name"]}: {1000 * dt:.2f} mm, {dr:.3f} deg')
    assert dt <= TOL['translation_m'] and dr <= TOL['rotation_deg']


def test_offsets_reach_the_requested_ranges():
    t = np.array([c.get('translation_m', [0, 0, 0]) for c in SPEC['cases']])
    a = np.array([c.get('rotation_deg', [0, 0, 0]) for c in SPEC['cases']])
    for axis in range(3):
        assert t[:, axis].min() == -0.2 and t[:, axis].max() == 0.2
        assert a[:, axis].min() == -90 and a[:, axis].max() == 90
