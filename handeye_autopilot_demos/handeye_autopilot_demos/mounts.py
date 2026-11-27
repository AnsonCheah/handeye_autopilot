"""Mount-sweep cases for the demos (no ROS imports; sweep_mounts runs them).

A case changes how the board (eye-to-hand) or the camera (eye-in-hand) is
bolted to the robot: an offset in the tool frame, translation added to the
demo's mount and rotation applied about the tool axes. The other object (the
camera on its tripod, the board on its stand) is then moved so that, at the
demo's seed joints, the camera sees the board exactly as in the original demo.
That keeps the seed a good seed (the operator's job on a real robot) while
the calibration has to recover a very different mount.

Frames as in the demo configs: ``true_camera_xyz_rpy`` is base -> camera_link
(eye-to-hand) or tool -> camera_link (eye-in-hand); ``true_board_xyz_rpy`` is
tool -> board (eye-to-hand) or base -> board (eye-in-hand); rpy extrinsic xyz
in radians.
"""

from __future__ import annotations

import copy
import math
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

Mat = np.ndarray


def pose(xyz_rpy: list[float]) -> Mat:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', xyz_rpy[3:]).as_matrix()
    T[:3, 3] = xyz_rpy[:3]
    return T


def xyz_rpy(T: Mat) -> list[float]:
    return [float(v) for v in T[:3, 3]] + [
        float(v) for v in Rotation.from_matrix(T[:3, :3]).as_euler('xyz')]


def inv(T: Mat) -> Mat:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def cases(spec: dict[str, Any]) -> list[dict[str, Any]]:
    """The case list: named cases, then ``random.count`` random ones (fixed seed)
    with each translation axis in +-translation_m and each rpy angle in
    +-rotation_deg. Each case: name, translation_m [3], rotation_deg [3]."""
    out = [{'name': c['name'], 'translation_m': list(c.get('translation_m', [0.0] * 3)),
            'rotation_deg': list(c.get('rotation_deg', [0.0] * 3))}
           for c in spec.get('cases', [])]
    r = spec.get('random')
    if r:
        rng = np.random.default_rng(r['seed'])
        for i in range(r['count']):
            t = rng.uniform(-r['translation_m'], r['translation_m'], 3)
            a = rng.uniform(-r['rotation_deg'], r['rotation_deg'], 3)
            out.append({'name': f'random{i + 1}', 'translation_m': [round(float(v), 3) for v in t],
                        'rotation_deg': [round(float(v), 1) for v in a]})
    return out


def moved_mount(mount: Mat, case: dict[str, Any]) -> Mat:
    """The robot-side mount with the case's offset (both in the tool frame)."""
    out = np.array(mount, dtype=float)
    R = Rotation.from_euler('xyz', np.radians(case['rotation_deg'])).as_matrix()
    out[:3, :3] = R @ out[:3, :3]
    out[:3, 3] = out[:3, 3] + np.asarray(case['translation_m'], float)
    return out


def case_config(cfg: dict[str, Any], mode: str, seed_tool: Mat,
                case: dict[str, Any], margin: float = 0.02) -> dict[str, Any]:
    """A copy of the robot's demo config with the case's mounts.

    ``seed_tool``: base -> tool at the seed joints (FK). The fixed object moves
    so the camera's view of the board at the seed is unchanged. The node's
    ``lever_max`` becomes the new lever arm's largest component plus
    ``margin``: what an operator would measure and enter.
    """
    out = copy.deepcopy(cfg)
    t = out[mode]
    cam, brd = pose(t['true_camera_xyz_rpy']), pose(t['true_board_xyz_rpy'])
    G = np.asarray(seed_tool, float)
    if mode == 'eye_to_hand':          # base_T_cam, tool_T_board
        view = inv(cam) @ G @ brd       # camera_T_board at the seed
        brd = moved_mount(brd, case)
        cam = G @ brd @ inv(view)
        lever = brd[:3, 3]
    else:                               # tool_T_cam, base_T_board
        view = inv(cam) @ inv(G) @ brd
        cam = moved_mount(cam, case)
        brd = G @ cam @ view
        lever = cam[:3, 3]
    t['true_camera_xyz_rpy'] = xyz_rpy(cam)
    t['true_board_xyz_rpy'] = xyz_rpy(brd)
    session = out.setdefault('calibration', {}).setdefault('session', {})
    session['lever_max'] = round(float(np.max(np.abs(lever))) + margin, 3)
    return out


def notes(cfg: dict[str, Any], mode: str) -> list[str]:
    """Physical-plausibility remarks about the moved fixed object (the run
    still happens: the synthetic camera renders no occlusion)."""
    r = cfg['robot']
    top = r['table']['xyz'][2] + r['table']['size'][2] / 2
    t = cfg[mode]
    fixed = pose(t['true_camera_xyz_rpy'] if mode == 'eye_to_hand'
                 else t['true_board_xyz_rpy'])
    out = []
    if fixed[2, 3] < top:
        what = 'camera' if mode == 'eye_to_hand' else 'board'
        out.append(f'fixed {what} {100 * (top - fixed[2, 3]):.0f} cm below the table top')
    return out


def view_angle_deg(cfg: dict[str, Any], mode: str, seed_tool: Mat) -> float:
    """Angle between the camera's optical axis and the line to the board at the
    seed (0 = board centred; only for tests and reports)."""
    t = cfg[mode]
    cam, brd = pose(t['true_camera_xyz_rpy']), pose(t['true_board_xyz_rpy'])
    G = np.asarray(seed_tool, float)
    view = inv(cam) @ G @ brd if mode == 'eye_to_hand' else inv(cam) @ inv(G) @ brd
    d = view[:3, 3] / np.linalg.norm(view[:3, 3])
    return math.degrees(math.acos(max(-1.0, min(1.0, float(d[0])))))  # camera_link: x forward
