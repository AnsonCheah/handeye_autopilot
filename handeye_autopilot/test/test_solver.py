"""M1b done-when: sweep, expected failure, blind spots, round trip (spec/m1b.yaml)."""

import math

from handeye_autopilot.cli import resolve
from handeye_autopilot.core import observations, solver
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.camera import CameraModel
from handeye_autopilot.core.config import SolverConfig
from handeye_autopilot.core.quality import calibrate
from m1b_sim import BOARD, CAMERA, SPEC, fast_run, rendered_run
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import yaml

MODES = ['eye_to_hand', 'eye_in_hand']
CFG = SolverConfig()
TOL = SPEC['sweep']['tolerance']
FRAMES = {'base_frame': 'base_link', 'tool_frame': 'tool0',
          'camera_frame': 'camera_optical_frame', 'board_frame': 'board'}


def errors(T, truth):
    return tf.translation_distance(T, truth), math.degrees(tf.rotation_angle(T, truth))


def assert_recovers(cal, X, Z):
    assert cal.transform_valid, cal.failure
    for est, true in ((cal.solution.X, X), (cal.solution.Z, Z)):
        t, r = errors(est, true)
        assert t <= TOL['translation_m'], f'{t * 1000:.2f} mm'
        assert r <= TOL['rotation_deg'], f'{r:.3f} deg'


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('seed', SPEC['sweep']['seeds'])
def test_sweep_recovers_ground_truth_and_passes(mode, seed):
    recs, X, Z = fast_run(mode, seed)
    cal = calibrate(mode, recs, CFG)
    assert_recovers(cal, X, Z)
    assert cal.passed, [g.name for g in cal.report.gates if g.gated and not g.passed]


@pytest.mark.parametrize('mode', MODES)
def test_end_to_end_through_renderer_and_detection(mode):
    r = SPEC['render']
    recs, X, Z = rendered_run(mode, r['seed'], r['captures'])
    assert sum(rec.accepted for rec in recs) >= CFG.min_captures
    cal = calibrate(mode, recs, CFG)
    assert_recovers(cal, X, Z)
    assert cal.passed


@pytest.mark.parametrize('mode', MODES)
def test_noise_free_is_exact(mode):
    from handeye_autopilot.core.render import RobotPoseNoise
    recs, X, Z = fast_run(mode, 1, pixel_noise=0.0, noise=RobotPoseNoise())
    sol = solver.solve(mode, recs, CFG)
    assert errors(sol.X, X)[0] < 1e-6 and errors(sol.Z, Z)[0] < 1e-6


@pytest.mark.parametrize('mode', MODES)
def test_parallel_rotation_axes_fail_with_their_code(mode):
    ef = SPEC['expected_failures']['parallel_axes']
    recs, *_ = fast_run(mode, 1, ranges=ef['poses'])
    cal = calibrate(mode, recs, CFG)
    assert not cal.transform_valid and not cal.passed
    assert cal.failure.reason == ef['reason']
    assert 'another axis' in str(cal.failure)
    assert math.isnan(cal.report.reprojection_rms_px)      # not computed -> NaN


def _blind(name, mode):
    bs = SPEC['blind_spots'][name]
    if 'object_scale' in bs:
        return fast_run(mode, 1, object_scale=bs['object_scale'])
    if 'focal_scale' in bs:
        K = CAMERA.K.copy()
        K[0, 0] *= bs['focal_scale']
        K[1, 1] *= bs['focal_scale']
        return fast_run(mode, 1, camera=CameraModel(K, CAMERA.D, CAMERA.width, CAMERA.height))
    S = tf.make(Rotation.from_rotvec(np.radians(bs['rotation_deg'])), bs['translation_m'])
    return fast_run(mode, 1, systematic=S)


@pytest.mark.parametrize('mode', MODES)
@pytest.mark.parametrize('name', list(SPEC['blind_spots']))
def test_known_blind_spots_pass_while_biased(mode, name, record_property, capsys):
    """02-features §H: shared errors stay internally consistent, so the gates
    can't see them. Asserts that they pass, and records the bias."""
    recs, X, _ = _blind(name, mode)
    cal = calibrate(mode, recs, CFG)
    assert cal.passed, [g.name for g in cal.report.gates if g.gated and not g.passed]
    t, r = errors(cal.solution.X, X)
    record_property(f'bias_{name}_{mode}', f'{t * 1000:.2f} mm, {r:.3f} deg')
    with capsys.disabled():
        print(f'\n  blind spot {name} ({mode}): camera transform off by '
              f'{t * 1000:.2f} mm, {r:.3f} deg, gates passed')
    if name == 'robot_systematic':
        # Its bias is below the noise here, so compare with the matched clean run
        # (same seed and noise): a right-multiplied tool bias S moves Z by inv(S)
        # in eye_to_hand and X by inv(S) in eye_in_hand.
        clean_recs, *_ = fast_run(mode, 1)
        clean = calibrate(mode, clean_recs, CFG).solution
        biased = cal.solution
        a, b = (clean.Z, biased.Z) if mode == 'eye_to_hand' else (clean.X, biased.X)
        shift = tf.translation_distance(a, b)
        bias = np.linalg.norm(SPEC['blind_spots'][name]['translation_m'])
        assert 0.5 * bias < shift < 2.0 * bias, shift
    else:
        assert t > TOL['translation_m'] / 2


@pytest.mark.parametrize('mode', MODES)
def test_save_reload_resolve_is_identical(mode, tmp_path):
    recs, *_ = fast_run(mode, 2)
    first = calibrate(mode, recs, CFG)
    path = tmp_path / 'observations.yaml'
    observations.save(path, recs, BOARD, {'mode': mode, 'frames': FRAMES})
    loaded, board, meta = observations.read(path)
    second = calibrate(meta['mode'], loaded, CFG)
    tol = SPEC['resolve_tolerance']
    for a, b in ((first.solution.X, second.solution.X), (first.solution.Z, second.solution.Z)):
        assert tf.translation_distance(a, b) <= tol and tf.rotation_angle(a, b) <= tol


@pytest.mark.parametrize('mode', MODES)
def test_resolve_cli(mode, tmp_path, capsys):
    recs, X, Z = fast_run(mode, 3)
    observations.save(tmp_path / 'observations.yaml', recs, BOARD,
                      {'mode': mode, 'frames': FRAMES})
    assert resolve.main([str(tmp_path)]) == 0
    out = yaml.safe_load((tmp_path / 'result_resolved.yaml').read_text())
    assert out['passed'] and out['mode'] == mode
    parents = {'eye_to_hand': ('base_link', 'tool0'), 'eye_in_hand': ('tool0', 'base_link')}
    expect = {'camera_transform': (parents[mode][0], 'camera_optical_frame', X),
              'target_transform': (parents[mode][1], 'board', Z)}
    for key, (parent, child, truth) in expect.items():
        d = out[key]
        assert (d['parent'], d['child']) == (parent, child)
        q = d['rotation_xyzw']                              # ROS order x, y, z, w
        T = tf.make(Rotation.from_quat(q), d['translation'])  # SciPy uses x, y, z, w too
        t, r = errors(T, truth)
        assert t <= TOL['translation_m'] and r <= TOL['rotation_deg']
        # a w-first reading of the same numbers must NOT give the truth
        wrong = Rotation.from_quat([q[3], q[0], q[1], q[2]])
        assert math.degrees((wrong.inv() * Rotation.from_matrix(truth[:3, :3])).magnitude()) > 1
    assert 'PASSED' in capsys.readouterr().out


def test_resolve_cli_reports_a_failed_solve(tmp_path, capsys):
    ef = SPEC['expected_failures']['parallel_axes']
    recs, *_ = fast_run('eye_to_hand', 1, ranges=ef['poses'])
    observations.save(tmp_path / 'observations.yaml', recs, BOARD,
                      {'mode': 'eye_to_hand', 'frames': FRAMES})
    assert resolve.main([str(tmp_path)]) == 1
    out = yaml.safe_load((tmp_path / 'result_resolved.yaml').read_text())
    assert out['transform_valid'] is False and out['quality']['reprojection_rms_px'] is None
    assert 'degenerate_rotation' in capsys.readouterr().out


def test_too_few_captures():
    recs, *_ = fast_run('eye_to_hand', 1, n=CFG.min_captures - 1)
    cal = calibrate('eye_to_hand', recs, CFG)
    assert cal.failure.reason == solver.TOO_FEW_CAPTURES


def test_a_bad_capture_is_removed_as_outlier():
    recs, X, Z = fast_run('eye_to_hand', 4)
    bad = recs[7]
    bad.tool_pose = bad.tool_pose @ tf.make(np.eye(3), [0.006, 0, 0])   # 6 mm wrong
    cal = calibrate('eye_to_hand', recs, CFG)
    assert cal.solution.outliers == [bad.capture_id]
    assert_recovers(cal, X, Z)
    # k-fold uses ALL accepted captures (no all-data outlier mask leaks in): the
    # bad capture is held out once and scored, so held-out max shows it.
    held = [c for f in cal.folds for c in f.held_out]
    assert sorted(held) == sorted(r.capture_id for r in recs)
    assert cal.report.heldout_reprojection_max_px > 3 * cal.report.reprojection_max_px


def test_failed_folds_fail_the_run_but_keep_the_transform():
    recs, *_ = fast_run('eye_in_hand', 1)
    cfg = SolverConfig(min_captures=len(recs) - 1)       # every training set is too small
    cal = calibrate('eye_in_hand', recs, cfg)
    assert cal.transform_valid and not cal.passed
    assert cal.report.folds_failed == cfg.folds
    gate = {g.name: g for g in cal.report.gates}
    assert not gate['folds_failed'].passed
    assert not gate['fold_translation_max'].passed     # NaN never passes
    assert math.isnan(cal.report.heldout_reprojection_rms_px)


def test_folds_are_index_modulo_k_in_acquisition_order():
    recs, *_ = fast_run('eye_to_hand', 2)
    cal = calibrate('eye_to_hand', recs, CFG)
    ids = sorted(r.capture_id for r in recs)
    for f in cal.folds:
        assert f.held_out == [c for i, c in enumerate(ids) if i % CFG.folds == f.index]


def test_axis_spread_definition():
    """§E4: 2 * lambda2 of mean(a a^T); parallel axes -> 0, two orthogonal -> 1."""
    recs, *_ = fast_run('eye_to_hand', 1)
    cfg = CFG

    def with_rotations(rotvecs):
        out = []
        for r, rv in zip(recs, rotvecs, strict=False):
            r2 = type(r).from_dict(r.to_dict())
            r2.tool_pose = tf.make(Rotation.from_rotvec(rv), [0, 0, 0])
            out.append(r2)
        return out
    a = math.radians(30)
    parallel = with_rotations([[0, 0, 0], [a, 0, 0], [2 * a, 0, 0]])
    assert solver.rotation_measures('eye_in_hand', parallel, cfg)[1] == pytest.approx(0, abs=1e-9)
    ortho = with_rotations([[0, 0, 0], [a, 0, 0], [0, a, 0]])
    amp, spread = solver.rotation_measures('eye_in_hand', ortho, cfg)
    # pairs: x-axis (a), y-axis (a), and the x->y pair; the mixed axis lies in the xy plane
    assert spread > 0.5 and amp == pytest.approx(math.radians(30) * math.sqrt(2), rel=0.05)


def test_failed_refinement_counts_as_a_failed_fold(monkeypatch):
    """A SciPy non-convergence is a SolveError, so the fold fails (never a silent pass)."""
    recs, *_ = fast_run('eye_in_hand', 2)
    real = solver.least_squares
    calls = {'n': 0}

    def flaky(*a, **k):
        calls['n'] += 1
        res = real(*a, **k)
        if calls['n'] > 1:          # the all-data fit works, every fold fit "fails"
            res.success = False
        return res
    monkeypatch.setattr(solver, 'least_squares', flaky)
    cal = calibrate('eye_in_hand', recs, CFG)
    assert cal.transform_valid and not cal.passed
    assert cal.report.folds_failed == CFG.folds
    assert all('refinement_failed' in f.reason for f in cal.folds)


@pytest.mark.parametrize('bad', [
    {'sigma_px': 0.0}, {'min_pair_rotation': 0.0}, {'folds': 2.5}, {'folds': 1},
    {'max_fold_rotation': float('nan')}, {'min_axis_spread': 1.5}, {'outlier_k': -1.0},
    {'sigma_px': True}, {'sigma_px': 1e-12},
])
def test_solver_config_rejects_values_that_break_the_math(bad):
    with pytest.raises(ValueError):
        SolverConfig(**bad)


def _reference_singular_value(mode, recs, X, Z, cfg):
    """Independent re-implementation of §E4 (perturbation applied on the right)."""
    from handeye_autopilot.core import handeye

    def resid(p):
        Xp = tf.make(X[:3, :3] @ Rotation.from_rotvec(p[0:3]).as_matrix(), X[:3, 3] + p[3:6])
        Zp = tf.make(Z[:3, :3] @ Rotation.from_rotvec(p[6:9]).as_matrix(), Z[:3, 3] + p[9:12])
        out = []
        for r in recs:
            C = handeye.predict_board(mode, Xp, Zp, r.tool_pose)
            out.append((r.camera.project_object(C, r.object_points) - r.image_points).ravel())
        return np.concatenate(out)
    r0 = resid(np.zeros(12))
    J = np.stack([(resid(np.eye(12)[k] * 1e-6) - resid(-np.eye(12)[k] * 1e-6)) / 2e-6
                  for k in range(12)], axis=1)
    s = cfg.f_scale_sigmas * cfg.sigma_px
    J = J * (np.sqrt(1 / np.sqrt(1 + (r0 / s) ** 2)) / cfg.sigma_px)[:, None]
    d = np.median([handeye.predict_board(mode, X, Z, r.tool_pose)[2, 3] for r in recs])
    J[:, 0:3] /= d
    J[:, 6:9] /= d
    return np.linalg.svd(J, compute_uv=False).min()


@pytest.mark.parametrize('offset', [[0, 0], [200, 150]])
def test_scaled_singular_value_matches_the_definition(offset):
    """Including off-axis boards, where range and optical-axis depth differ."""
    ranges = dict(SPEC['poses'], offset=[offset[0] - 5, offset[0] + 5])
    recs, X, Z = fast_run('eye_to_hand', 1, ranges=ranges)
    got = solver.scaled_min_singular_value('eye_to_hand', recs, X, Z, CFG)
    assert got == pytest.approx(_reference_singular_value('eye_to_hand', recs, X, Z, CFG),
                                rel=1e-4)


def test_more_folds_than_captures_fail_verification_not_the_transform():
    recs, *_ = fast_run('eye_to_hand', 1, n=10)
    cfg = SolverConfig(folds=11)
    cal = calibrate('eye_to_hand', recs, cfg)
    assert cal.transform_valid and not cal.passed
    assert cal.report.folds_failed == 11


@pytest.mark.parametrize('text', ['[]', 'sigma_px: [', '- 1\n- 2'])
def test_resolve_cli_rejects_bad_config(tmp_path, text, capsys):
    recs, *_ = fast_run('eye_in_hand', 3)
    observations.save(tmp_path / 'observations.yaml', recs, BOARD,
                      {'mode': 'eye_in_hand', 'frames': FRAMES})
    (tmp_path / 'cfg.yaml').write_text(text)
    assert resolve.main([str(tmp_path), '-c', str(tmp_path / 'cfg.yaml')]) == 2
    assert 'error' in capsys.readouterr().err
