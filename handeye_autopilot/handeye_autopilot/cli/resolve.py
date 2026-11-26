"""Re-solve a recorded run offline (02-features §G2).

  ros2 run handeye_autopilot resolve <run_dir | observations.yaml> [-c solver.yaml] [-o out.yaml]

Reads the observations YAML (schema_version checked), runs the same solver,
k-fold verification and gates as the node, prints a summary in mm/degrees and
writes a result YAML (default: <run_dir>/result_resolved.yaml).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import yaml

from handeye_autopilot.core import observations
from handeye_autopilot.core.config import SolverConfig
from handeye_autopilot.core.quality import calibrate
from handeye_autopilot.core.results import result_dict


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run', help='run directory (with observations.yaml) or the file itself')
    p.add_argument('-c', '--config', help='solver config YAML (SolverConfig fields)')
    p.add_argument('-o', '--output', help='result YAML path')
    a = p.parse_args(argv)
    path = Path(a.run)
    obs_file = path / 'observations.yaml' if path.is_dir() else path
    try:
        records, board, meta = observations.read(obs_file)
        cfg = SolverConfig()
        if a.config:
            raw = yaml.safe_load(Path(a.config).read_text())
            if raw is not None and not isinstance(raw, dict):
                raise ValueError(f'{a.config}: expected a mapping of SolverConfig fields')
            cfg = SolverConfig.from_dict(raw or {})
    except (OSError, ValueError, yaml.YAMLError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    cal = calibrate(meta['mode'], records, cfg)
    frames = dict(meta['frames'])
    out = Path(a.output) if a.output else obs_file.parent / 'result_resolved.yaml'
    out.write_text(yaml.safe_dump(result_dict(cal, cfg, frames, meta), sort_keys=False))

    r = cal.report
    print(f'mode {cal.mode}: {r.captures_accepted} accepted captures, '
          f'{r.inliers} inliers, {r.outliers} outliers')
    if cal.solution is None:
        assert cal.failure is not None
        print(f'NO RESULT ({cal.failure.reason}): {cal.failure}')
        print(f'wrote {out}')
        return 1
    t = cal.solution.X[:3, 3] * 1000
    print(f'camera transform: [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}] mm')
    for g in r.gates:
        unit, val, lim = g.unit, g.value, g.limit
        if unit == 'rad':
            unit, val, lim = 'deg', math.degrees(val), math.degrees(lim)
        elif unit == 'm':
            unit, val, lim = 'mm', val * 1000, lim * 1000
        if not g.comparison:
            print(f'  info {g.group:17s} {g.name:26s} {val:10.4f} {unit}')
            continue
        mark = 'PASS' if g.passed else 'FAIL'
        print(f'  {mark:4s} {g.group:17s} {g.name:26s} {val:10.4f} {g.comparison} '
              f'{lim:.4f} {unit}')
    print('PASSED' if cal.passed else 'QUALITY GATES FAILED', f'- wrote {out}')
    return 0 if cal.passed else 1


if __name__ == '__main__':
    sys.exit(main())
