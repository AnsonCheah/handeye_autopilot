"""Bootstrap step checks (02-features §B2): numeric, worst case over the lever arm.

Before the first solve the only geometric knowledge is the measured board pose
C and the bound L_max on the lever arm between the tool and the moving object
(the board for eye_to_hand, the camera for eye_in_hand), given per tool axis.
The tool's orientation in the camera frame is unknown too, so the tool origin
(the pivot of a tool rotation) can be anywhere in the ball of radius
sqrt(3) * L_max around the moving object's reference point (the board centre,
or the camera origin), and the step's axis or direction can be anything.

In the camera frame a tool step therefore shows up as an unknown rigid motion
of the board: for eye_to_hand the board moves with the tool (forward motion);
for eye_in_hand the camera moves and the board appears to move by the inverse.
Both are covered by sampling both signs of every axis/direction. The check:

- axes / directions: a dense Fibonacci sphere (both signs);
- pivots: the reference point and a Fibonacci sphere of radius sqrt(3) L_max;
- the whole motion, not only its end: fractions 1/4 ... 1 of step x safety;
- every board-outline sample projected with the full camera model; all must
  stay in front of the camera and inside the image with the margin.

shortcut: sampled, not exact; a worst case between samples is covered by the
safety factor and the margin; switch to an analytic bound if a real run ever
loses the board after a step this check passed.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from . import transforms as tf
from .boards import BoardSpec
from .camera import CameraModel
from .config import SessionConfig

Mat = NDArray[np.float64]
FRACTIONS = (0.25, 0.5, 0.75, 1.0)


def fibonacci_sphere(n: int) -> Mat:
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5 ** 0.5) * i
    pts = np.c_[np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)]
    return np.asarray(pts, dtype=np.float64)


DIRECTIONS = fibonacci_sphere(64)          # both signs included (whole sphere)
PIVOT_DIRS = fibonacci_sphere(32)


def reference_point(mode: str, C: Mat, board: BoardSpec) -> Mat:
    """Camera-frame point the lever arm is measured from: board centre
    (eye_to_hand) or the camera origin (eye_in_hand)."""
    if mode == 'eye_to_hand':
        w, h = board.size_m
        return np.asarray(tf.apply(C, np.array([[w / 2, h / 2, 0.0]]))[0], dtype=np.float64)
    if mode == 'eye_in_hand':
        return np.zeros(3)
    raise ValueError(f'unknown mode {mode!r}')


pivot_origin = reference_point          # name used by earlier callers


def _visible_all(pts: Mat, camera: CameraModel, margin: float) -> bool:
    """pts: (..., 3) camera-frame points; True if every one is in view with the margin."""
    p = pts.reshape(-1, 3)
    if np.any(p[:, 2] <= 1e-3):
        return False
    px = camera.project(p)
    return bool(px[:, 0].min() >= margin and px[:, 1].min() >= margin and
                px[:, 0].max() <= camera.width - 1 - margin and
                px[:, 1].max() <= camera.height - 1 - margin)


def _faces_camera(normals: Mat, centres: Mat) -> bool:
    """Printed side toward the camera for every (normal, centre) pair: the board
    +z axis points away from the camera, i.e. along the line of sight (same
    rule as planning.visibility)."""
    return bool(np.all(np.einsum('...i,...i->...', normals, centres) > 0))


def rotation_is_safe(mode: str, C: Mat, board: BoardSpec, camera: CameraModel,
                     angle: float, cfg: SessionConfig, directions: Mat = DIRECTIONS,
                     pivot_dirs: Mat = PIVOT_DIRS,
                     fractions: tuple[float, ...] = FRACTIONS) -> bool:
    pts = tf.apply(C, board.outline_samples(12))                     # (P, 3)
    w, h = board.size_m
    centre = tf.apply(C, np.array([[w / 2, h / 2, 0.0]]))[0]
    ref = reference_point(mode, C, board)
    pivots = np.vstack([ref, ref + np.sqrt(3) * cfg.lever_max * pivot_dirs])
    for frac in fractions:
        R = Rotation.from_rotvec(directions * angle * cfg.step_safety * frac).as_matrix()
        normals = R @ C[:3, 2]                                           # (A, 3)
        for pivot in pivots:
            moved = np.einsum('aij,pj->api', R, pts - pivot) + pivot   # (A, P, 3)
            if not _visible_all(moved, camera, cfg.image_margin_px):
                return False
            if not _faces_camera(normals, R @ (centre - pivot) + pivot):
                return False
    return True


def translation_is_safe(C: Mat, board: BoardSpec, camera: CameraModel, length: float,
                        cfg: SessionConfig, directions: Mat = DIRECTIONS,
                        fractions: tuple[float, ...] = FRACTIONS) -> bool:
    pts = tf.apply(C, board.outline_samples(12))
    for frac in fractions:
        moved = pts[None] + (directions * length * cfg.step_safety * frac)[:, None]
        if not _visible_all(moved, camera, cfg.image_margin_px):
            return False
    return True


def predicted_is_safe(mode: str, X: Mat, Z: Mat, start_tool: Mat, target_tool: Mat,
                      board: BoardSpec, camera: CameraModel, margin: float) -> bool:
    """With a rough estimate: is the board predicted in view along the whole move?"""
    from .handeye import predict_board
    pts = board.outline_samples(12)
    for frac in FRACTIONS:          # the same straight path the Cartesian waypoints follow
        C = predict_board(mode, X, Z, tf.interpolate(start_tool, target_tool, frac))
        w, h = board.size_m
        if not _faces_camera(C[:3, 2], tf.apply(C, np.array([[w / 2, h / 2, 0.0]]))[0]):
            return False
        if not _visible_all(tf.apply(C, pts), camera, margin):
            return False
    return True


def safe_step(check: Callable[[float], bool], start: float, minimum: float) -> float:
    """Largest step <= start (halving) that passes ``check(step)``; 0 if none >= minimum."""
    step = start
    while step >= minimum:
        if check(step):
            return step
        step *= 0.5
    return 0.0


def tool_step(kind: str, axis: int, value: float) -> Mat:
    """A step in the tool frame: rotation about / translation along tool axis ``axis``."""
    e = np.zeros(3)
    e[axis] = 1.0
    if kind == 'rotation':
        return tf.make(Rotation.from_rotvec(value * e), np.zeros(3))
    return tf.make(np.eye(3), value * e)
