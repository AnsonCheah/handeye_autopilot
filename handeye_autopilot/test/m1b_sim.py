"""Ground-truth hand-eye runs for the M1b tests (spec/m1b.yaml).

The oracle builds tool poses from the physical chain only, independent of the
mode adapter:
  eye_to_hand: camera_T_board = inv(base_T_camera) . base_T_tool . tool_T_board
               -> base_T_tool = X . C . inv(Z)
  eye_in_hand: camera_T_board = inv(tool_T_camera) . inv(base_T_tool) . base_T_board
               -> base_T_tool = Z . inv(C) . inv(X)
"""

from __future__ import annotations

import math
from pathlib import Path

from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.boards import BoardSpec, board_texture
from handeye_autopilot.core.camera import CameraModel
from handeye_autopilot.core.capture import CaptureRecord, Frame, capture
from handeye_autopilot.core.config import CaptureConfig
from handeye_autopilot.core.pose import solve as solve_pnp
from handeye_autopilot.core.render import (
    ImageEffects,
    PlanarObject,
    Renderer,
    RobotPoseNoise,
    look_at_board,
)
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

SPEC = yaml.safe_load((Path(__file__).parent / 'spec' / 'm1b.yaml').read_text())
CAMERA = CameraModel.from_dict(SPEC['camera'])
BOARD = BoardSpec.from_dict(SPEC['board'])


def T_of(d) -> np.ndarray:
    return tf.make(Rotation.from_euler('xyz', d['rpy'], degrees=True), d['t'])


def truth(mode):
    return T_of(SPEC['truth'][mode]['X']), T_of(SPEC['truth'][mode]['Z'])


def tool_from_board(mode, X, Z, C):
    return X @ C @ tf.inv(Z) if mode == 'eye_to_hand' else Z @ tf.inv(C) @ tf.inv(X)


def board_poses(rng, n, ranges=None):
    r = ranges or SPEC['poses']
    w, h = BOARD.size_m
    out = []
    for _ in range(n):
        tilt = np.radians(rng.uniform(*r['tilt'], 2))
        out.append(look_at_board(np.array([w / 2, h / 2, 0]), rng.uniform(*r['distance']),
                                 tilt[0], tilt[1], math.radians(rng.uniform(*r['roll'])),
                                 offset_px=tuple(rng.uniform(*r['offset'], 2)), camera=CAMERA))
    return out


def robot_noise(cfg=None):
    rn = cfg or SPEC['sweep']['robot_noise']
    return RobotPoseNoise(rn['translation_m'], math.radians(rn['rotation_deg']))


def fast_run(mode, seed, n=None, pixel_noise=None, ranges=None, noise=None,
             object_scale=1.0, camera=None, systematic=None):
    """Capture records from projected corners + noise (no rendering).

    object_scale / camera / systematic inject the blind-spot errors: the
    records carry the WRONG board size / intrinsics / robot pose while the
    pixels come from the truth.
    """
    rng = np.random.default_rng(seed)
    X, Z = truth(mode)
    n = n or SPEC['sweep']['captures']
    sigma = SPEC['sweep']['pixel_noise_px'] if pixel_noise is None else pixel_noise
    rn = noise or robot_noise()
    if systematic is not None:
        rn.systematic = systematic
    cam_used = camera or CAMERA
    obj = BOARD.corner_points
    recs = []
    for i, C in enumerate(board_poses(rng, n, ranges)):
        G = tool_from_board(mode, X, Z, C)
        px = CAMERA.project_object(C, obj) + rng.normal(0, sigma, (len(obj), 2))
        used_obj = obj * object_scale
        p = solve_pnp(used_obj, px, cam_used)
        recs.append(CaptureRecord(
            i, float(i), cam_used, np.arange(len(obj), dtype=np.int32), used_obj, px,
            rn.report(G, rng), p.best.camera_T_board,
            [h.camera_T_board for h in p.hypotheses], [h.rms_px for h in p.hypotheses],
            True))
    return recs, X, Z


def rendered_run(mode, seed, n):
    """Capture records through the renderer and the M1a detection + gates."""
    rng = np.random.default_rng(seed)
    X, Z = truth(mode)
    tex, H = board_texture(BOARD, 60, 30)
    renderer = Renderer(CAMERA, ImageEffects(noise_sigma=2.0))
    cfg = CaptureConfig(consensus_min_frames=1, max_failed_frames=0)
    rn = robot_noise()
    recs = []
    for i, C in enumerate(board_poses(rng, n)):
        G = tool_from_board(mode, X, Z, C)
        img = renderer.render([PlanarObject(tex, H, C)], rng)
        recs.append(capture(i, [Frame(img, float(i), rn.report(G, rng))], BOARD, CAMERA, cfg))
    return recs, X, Z
