"""Mount-sweep case geometry (handeye_autopilot_demos.mounts), no ROS needed."""

import math
from pathlib import Path

from handeye_autopilot_demos.sweep_mounts import parse
import numpy as np
import pytest
import yaml

from handeye_autopilot_demos import mounts

CONFIG = Path(__file__).resolve().parents[1] / 'config'
SEED = mounts.pose([0.3, 0.05, 0.45, 3.0, 0.1, -0.4])     # any base -> tool pose


def _cfg(robot='panda'):
    return yaml.safe_load((CONFIG / f'{robot}.yaml').read_text())


def _view(cfg, mode, G):
    t = cfg[mode]
    cam, brd = mounts.pose(t['true_camera_xyz_rpy']), mounts.pose(t['true_board_xyz_rpy'])
    return (mounts.inv(cam) @ G @ brd if mode == 'eye_to_hand'
            else mounts.inv(cam) @ mounts.inv(G) @ brd)


def test_case_list_covers_the_requested_ranges():
    spec = yaml.safe_load((CONFIG / 'mount_sweep.yaml').read_text())
    cs = mounts.cases(spec)
    assert len({c['name'] for c in cs}) == len(cs)
    t = np.array([c['translation_m'] for c in cs])
    a = np.array([c['rotation_deg'] for c in cs])
    assert np.abs(t).max() == pytest.approx(0.2) and np.abs(a).max() == pytest.approx(90)
    assert np.all(np.abs(t) <= 0.2) and np.all(np.abs(a) <= 90)
    for axis in range(3):            # each axis reaches both extremes
        assert t[:, axis].min() == pytest.approx(-0.2) and t[:, axis].max() == pytest.approx(0.2)
        assert a[:, axis].min() == pytest.approx(-90) and a[:, axis].max() == pytest.approx(90)
    assert mounts.cases(spec) == cs  # random cases are reproducible


@pytest.mark.parametrize('robot', ['panda', 'ur'])
@pytest.mark.parametrize('mode', ['eye_to_hand', 'eye_in_hand'])
def test_case_moves_the_robot_side_mount_and_keeps_the_seed_view(robot, mode):
    cfg = _cfg(robot)
    case = {'name': 'x', 'translation_m': [0.2, -0.1, 0.05], 'rotation_deg': [90, -30, 45]}
    out = mounts.case_config(cfg, mode, SEED, case)
    key = 'true_board_xyz_rpy' if mode == 'eye_to_hand' else 'true_camera_xyz_rpy'
    before, after = mounts.pose(cfg[mode][key]), mounts.pose(out[mode][key])
    np.testing.assert_allclose(after[:3, 3] - before[:3, 3], case['translation_m'], atol=1e-9)
    R = mounts.pose([0, 0, 0] + [math.radians(v) for v in case['rotation_deg']])[:3, :3]
    np.testing.assert_allclose(after[:3, :3], R @ before[:3, :3], atol=1e-9)
    np.testing.assert_allclose(_view(out, mode, SEED), _view(cfg, mode, SEED), atol=1e-9)
    b = cfg['board']                     # the centre, written out (not the helper under test)
    ref = (np.array([b['squares_x'], b['squares_y'], 0.0]) * b['square_length'] / 2
           if mode == 'eye_to_hand' else np.zeros(3))
    lever = (after @ np.r_[ref, 1.0])[:3]            # tool -> board centre / camera origin
    assert out['calibration']['session']['lever_max'] == pytest.approx(
        np.abs(lever).max() + 0.02, abs=1e-3)
    assert cfg[mode][key] != out[mode][key]          # the input config is not modified


def test_zero_case_changes_nothing_but_the_lever():
    cfg = _cfg()
    out = mounts.case_config(cfg, 'eye_to_hand', SEED,
                             {'name': 'z', 'translation_m': [0, 0, 0], 'rotation_deg': [0, 0, 0]})
    for k in ('true_camera_xyz_rpy', 'true_board_xyz_rpy'):
        np.testing.assert_allclose(mounts.pose(out['eye_to_hand'][k]),
                                   mounts.pose(cfg['eye_to_hand'][k]), atol=1e-9)


def test_notes_flag_a_fixed_object_below_the_table():
    cfg = _cfg()
    cfg['eye_in_hand']['true_board_xyz_rpy'][2] = -0.3
    assert 'below the table top' in mounts.notes(cfg, 'eye_in_hand')[0]
    assert mounts.notes(_cfg(), 'eye_in_hand') == []


def test_parse_reads_the_runner_output():
    ok = ('[demo_runner]: finished: status 4, error_code 0: calibration passed all quality gates\n'
          '[demo_runner]: captures accepted 60, rejected 1; reprojection RMS 0.24 px\n'
          '[demo_runner]: error against the true camera mount: 0.02 mm, 0.005 deg\n'
          '[demo_runner]: DEMO PASSED\n')
    r = parse(ok, 0)
    assert r['verdict'] == 'passed' and r['error_mm'] == 0.02 and r['accepted'] == 60
    assert parse(ok.replace('PASSED', 'FAILED'), 1)['verdict'] == 'failed'
    seed = ('could not reach the seed: status 6, START_STATE_IN_COLLISION \n'
            'contacts at the current state: calibration_board <-> panda_link6\n'
            'contacts at the seed: none\n')
    r = parse(seed, 1)
    assert r['verdict'] == 'setup' and 'calibration_board <-> panda_link6' in r['message']
    # GOAL_IN_COLLISION is explained by the seed's contacts, not the current state's
    goal = seed.replace('START_STATE_IN_COLLISION', 'GOAL_IN_COLLISION')
    assert parse(goal, 1)['verdict'] == 'failed'
    assert parse(goal.replace('seed: none', 'seed: camera <-> wrist_1_link'), 1)[
        'verdict'] == 'setup'
    # a collision the mount is not part of (robot and table) is a failure, even if
    # the board collides at the other state
    other = seed.replace('calibration_board <-> panda_link6', 'panda_link6 <-> table')
    assert parse(other, 1)['verdict'] == 'failed'
    assert parse(other.replace('seed: none', 'seed: calibration_board <-> panda_link6'), 1)[
        'verdict'] == 'failed'
    assert parse(seed.split('contacts')[0], 1)['verdict'] == 'failed'    # no contacts
    # no collision evidence, or a timeout: a failure, never excluded as setup
    assert parse(seed.replace('START_STATE_IN_COLLISION', 'CONTROL_FAILED'), 1)[
        'verdict'] == 'failed'
    assert parse(seed, None)['verdict'] == 'failed'
    late = ('the calibration did not finish in time; cancelling it\n'
            'the run ended: status 6, error_code 9: motion_fault: stop not confirmed\n')
    assert parse(late, 1)['message'] == 'motion_fault: stop not confirmed'
    assert parse('', None) == {'exit_code': None, 'verdict': 'failed', 'message': 'timed out'}
