"""Candidate pose planning (02-features §C): sample, filter, select, order.

Reuse: SciPy qmc.Halton, OpenCV projectPoints (via CameraModel), the mode
adapter for camera-relative board pose <-> tool pose, the solver's scaled
Jacobian for the information check. IK is injected (MoveIt compute_ik in the
node, a fake in tests) and every rejection is recorded with its reason.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from scipy.stats import qmc

from . import handeye
from . import transforms as tf
from .boards import BoardSpec
from .camera import CameraModel
from .capture import CaptureRecord
from .config import SessionConfig, SolverConfig
from .gates import outline_margin
from .solver import rotation_measures, scaled_min_singular_value

Mat = NDArray[np.float64]

# Candidate rejection reasons
NOT_VISIBLE = 'not_visible'
BACK_FACING = 'back_facing'
COVERAGE = 'coverage'
TOO_SMALL = 'features_too_small'
OUTSIDE_BOX = 'outside_box'
OUTSIDE_ENVELOPE = 'outside_envelope'
ORIENTATION_LIMIT = 'orientation_limit'
IK_FAILED = 'ik_failed'


@dataclass
class IKResult:
    joints: Mat | None
    reason: str = ''          # e.g. the MoveIt error code when joints is None


IKFunction = Callable[[Mat], IKResult]


@dataclass(eq=False)          # identity: candidates hold arrays
class Candidate:
    camera_T_board: Mat
    tool_pose: Mat = field(default_factory=lambda: np.eye(4))
    joints: Mat | None = None
    reject: str | None = None
    detail: str = ''


@dataclass
class Plan:
    poses: list[Candidate]                 # ordered for execution (selected + spares)
    target: int                            # accepted captures to collect from this plan
    information: float                     # scaled smallest singular value of the plan
    weak: bool                             # information below the threshold
    rejections: Counter[str] = field(default_factory=Counter)


# -- 1. sampling ----------------------------------------------------------

def sample(camera: CameraModel, board: BoardSpec, cfg: SessionConfig, seed: int = 0
           ) -> list[Mat]:
    """Camera-relative board poses: centre pixel -> ray, depth, two tilts, roll (§C1)."""
    s = qmc.Halton(d=6, scramble=True, seed=seed).random(cfg.candidates)
    b = cfg.image_border_frac
    u = camera.width * (b + (1 - 2 * b) * s[:, 0])
    v = camera.height * (b + (1 - 2 * b) * s[:, 1])
    rays = np.c_[camera.normalized(np.c_[u, v]), np.ones(len(u))]
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    depth = cfg.depth_range[0] + (cfg.depth_range[1] - cfg.depth_range[0]) * s[:, 2]
    tx, ty = (2 * s[:, 3] - 1) * cfg.max_tilt, (2 * s[:, 4] - 1) * cfg.max_tilt
    roll = (2 * s[:, 5] - 1) * cfg.max_roll
    w, h = board.size_m
    centre = np.array([w / 2, h / 2, 0.0])
    out = []
    for k in range(len(u)):
        R = Rotation.from_euler('z', roll[k]) * Rotation.from_euler('xy', [tx[k], ty[k]])
        p = rays[k] * depth[k] / rays[k][2]      # depth along the optical axis
        out.append(tf.make(R, p - R.apply(centre)))
    return out


# -- 2. visibility --------------------------------------------------------

def visibility(C: Mat, camera: CameraModel, board: BoardSpec, cfg: SessionConfig,
               uncertainty_px: float | None = None) -> tuple[str | None, str]:
    """§C2: in front, printed face toward the camera, outline inside with margin
    plus the estimate-uncertainty term, coverage band, feature size."""
    w, h = board.size_m
    centre = tf.apply(C, np.array([[w / 2, h / 2, 0.0]]))[0]
    if centre[2] <= 0:
        return NOT_VISIBLE, 'behind the camera'
    if C[:3, 2] @ centre <= 0:               # board +z must point away from the camera
        return BACK_FACING, 'printed side faces away'
    margin = outline_margin(C, board, camera)
    unc = cfg.uncertainty_margin_px if uncertainty_px is None else uncertainty_px
    need = cfg.image_margin_px + unc
    if margin < need:
        return NOT_VISIBLE, f'outline {margin:.0f} px from the edge < {need:.0f}'
    # coverage from the densely sampled outline (edges curve under distortion)
    px = camera.project_object(C, board.outline_samples(50))
    area = 0.5 * abs(np.dot(px[:, 0], np.roll(px[:, 1], 1)) -
                     np.dot(px[:, 1], np.roll(px[:, 0], 1)))
    frac = area / (camera.width * camera.height)
    if not cfg.coverage[0] <= frac <= cfg.coverage[1]:
        return COVERAGE, f'board covers {frac:.2f} of the image'
    cols, rows = board.inner_corners
    grid = camera.project_object(C, board.corner_points).reshape(rows, cols, 2)
    spacing = min(np.linalg.norm(np.diff(grid, axis=0), axis=2).min(),
                  np.linalg.norm(np.diff(grid, axis=1), axis=2).min())
    if spacing < cfg.min_corner_spacing_px:
        return TOO_SMALL, f'corner spacing {spacing:.1f} px'
    if board.kind == 'charuco':
        module = marker_module_px(C, camera, board)
        if module < cfg.min_marker_module_px:
            return TOO_SMALL, f'marker module {module:.1f} px'
    return None, ''


def marker_module_px(C: Mat, camera: CameraModel, board: BoardSpec) -> float:
    """Smallest projected marker bit (px): each marker's shortest projected edge
    divided by its cells (inner bits from the OpenCV dictionary + 2 border).
    shortcut: the average cell along an edge, not the smallest foreshortened
    cell; fine while markers are small against the viewing distance."""
    from . import cv_compat
    bits = cv_compat.marker_bits(cv_compat.get_dictionary(board.dictionary)) + 2
    _, obj = cv_compat.marker_object_points(board.opencv_board)            # (M, 4, 3)
    px = camera.project_object(C, obj.reshape(-1, 3)).reshape(-1, 4, 2)
    edges = np.linalg.norm(px - np.roll(px, 1, axis=1), axis=2)
    return float(edges.min() / bits)


# -- 4. robot-agnostic limits ----------------------------------------------

def box_contains(points_base: Mat, cfg: SessionConfig) -> bool:
    """Are all points (base frame) inside the posed safety box?"""
    B = tf.make(Rotation.from_euler('xyz', cfg.box_rpy), np.asarray(cfg.box_xyz, dtype=float))
    local = tf.apply(tf.inv(B), np.atleast_2d(points_base))
    half = np.asarray(cfg.box_size) / 2
    return bool(np.all(np.abs(local) <= half))


def limits(G: Mat, seed_tool: Mat, cfg: SessionConfig,
           moving_points: Mat | None = None) -> tuple[str | None, str]:
    """Safety box (toggle), envelope around the seed, orientation limit from the
    seed. The envelope and orientation limits always apply; the box adds a
    user-placed volume on top (01-architecture §3.4)."""
    p = G[:3, 3]
    if cfg.box_enabled:
        pts = p[None] if moving_points is None or not cfg.box_check_moving_object else \
            np.vstack([p[None], moving_points])
        if not box_contains(pts, cfg):
            return OUTSIDE_BOX, 'outside the safety box'
    d = float(np.linalg.norm(p - seed_tool[:3, 3]))
    if d > cfg.envelope_radius:
        return OUTSIDE_ENVELOPE, f'{d:.2f} m from the seed > {cfg.envelope_radius}'
    a = tf.rotation_angle(G, seed_tool)
    if a > cfg.max_orientation_change:
        return ORIENTATION_LIMIT, f'{np.degrees(a):.0f} deg from the seed orientation'
    return None, ''


def moving_points_base(mode: str, G: Mat, X: Mat, Z: Mat, board: BoardSpec) -> Mat:
    """Base-frame points of the carried object: board corners (eye_to_hand) or the
    camera origin (eye_in_hand), for the optional box check."""
    if mode == 'eye_to_hand':
        return tf.apply(G @ Z, board.outline)
    return (G @ X)[:3, 3][None]


# -- 6. information --------------------------------------------------------

def predicted_records(mode: str, X: Mat, Z: Mat, tool_poses: list[Mat],
                      camera: CameraModel, board: BoardSpec) -> list[CaptureRecord]:
    """Noise-free records the estimate predicts for these tool poses."""
    out = []
    obj = board.corner_points
    ids = np.arange(len(obj), dtype=np.int32)
    for k, G in enumerate(tool_poses):
        C = handeye.predict_board(mode, X, Z, G)
        px = camera.project_object(C, obj)
        out.append(CaptureRecord(k, float(k), camera, ids, obj, px, G, C, [C, C],
                                 [0.0, 0.0], True))
    return out


def information(mode: str, X: Mat, Z: Mat, tool_poses: list[Mat], camera: CameraModel,
                board: BoardSpec, solver_cfg: SolverConfig) -> float:
    """Scaled smallest singular value the set would give (§E4), or 0 when the set
    would already fail the solver's rotation checks (amplitude, axis spread):
    information alone can look fine for tiny rotations the solver then rejects."""
    if len(tool_poses) < 3:
        return 0.0
    recs = predicted_records(mode, X, Z, tool_poses, camera, board)
    amp, spread = rotation_measures(mode, recs, solver_cfg)
    if amp < solver_cfg.min_rotation_amplitude or spread < solver_cfg.min_axis_spread:
        return 0.0
    return scaled_min_singular_value(mode, recs, X, Z, solver_cfg)


def _relative(Ra: Mat, Rb: Mat) -> tuple[Mat, Mat]:
    """Angles (...,) and unit axes (..., 3) of Ra^T Rb, batched over leading dims."""
    rel = np.einsum('...ji,...jk->...ik', Ra, Rb)
    cos = np.clip((np.trace(rel, axis1=-2, axis2=-1) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(cos)
    v = np.stack([rel[..., 2, 1] - rel[..., 1, 2], rel[..., 0, 2] - rel[..., 2, 0],
                  rel[..., 1, 0] - rel[..., 0, 1]], axis=-1)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    axes = np.where(n > 1e-12, v / np.maximum(n, 1e-12), 0.0)
    # near a half-turn the skew part vanishes; take the axis from SciPy there
    bad = (n[..., 0] < 1e-6) & (ang > np.pi / 2)
    if np.any(bad):
        rv = Rotation.from_matrix(rel[bad]).as_rotvec()
        axes[bad] = rv / np.linalg.norm(rv, axis=-1, keepdims=True)
    return ang, axes


def select(mode: str, cands: list[Candidate], existing: list[Mat], cfg: SessionConfig,
           count: int | None = None) -> list[Candidate]:
    """Greedy selection over relative rotations of the moving rotations A (§C6).

    Score = (smallest SO(3) angle to anything chosen or captured) x
    (1 + axis novelty), where axis novelty = 1 - max |cos| between the new
    relative rotation axis (to its nearest chosen pose) and the axes already
    used; so the set spreads in angle AND in axis direction.
    """
    n = count if count is not None else int(np.ceil(cfg.target_captures * cfg.reserve_factor))
    pool = list(cands)
    if not pool or n <= 0:
        return []
    P = np.array([handeye.moving(mode, c.tool_pose)[:3, :3] for c in pool])   # (N, 3, 3)
    chosen_R = [handeye.moving(mode, G)[:3, :3] for G in existing]
    axes: list[Mat] = []
    if len(chosen_R) > 1:
        E = np.array(chosen_R)
        ang, ax = _relative(E[:, None], E[None, :])
        iu = np.triu_indices(len(E), 1)
        axes = [a for a, t in zip(ax[iu], ang[iu], strict=True) if t > 1e-6]
    alive = np.ones(len(pool), dtype=bool)
    out: list[Candidate] = []
    while alive.any() and len(out) < n:
        if not chosen_R:
            k = int(np.flatnonzero(alive)[0])
        else:
            ang, ax = _relative(np.array(chosen_R)[None, :], P[:, None])       # (N, K)
            near = np.argmin(ang, axis=1)
            a_min = ang[np.arange(len(P)), near]
            ax_n = ax[np.arange(len(P)), near]
            if axes:
                novelty = 1.0 - np.max(np.abs(ax_n @ np.array(axes).T), axis=1)
            else:
                novelty = np.ones(len(P))
            score = np.where(alive, a_min * (1.0 + novelty), -np.inf)
            k = int(np.argmax(score))
            if a_min[k] > 1e-6:
                axes.append(ax_n[k])
        alive[k] = False
        out.append(pool[k])
        chosen_R.append(P[k])
    return out


# -- 8. ordering -----------------------------------------------------------

def order(cands: list[Candidate], start_joints: Mat | None) -> list[Candidate]:
    """Greedy nearest-neighbour tour in joint space from the current joints."""
    if not cands or cands[0].joints is None:
        return list(cands)
    left = list(cands)
    cur = start_joints if start_joints is not None else left[0].joints
    out = []
    while left:
        nxt = min(left, key=lambda c: float(np.linalg.norm(c.joints - cur)))  # type: ignore[operator]
        left.remove(nxt)
        out.append(nxt)
        cur = nxt.joints
    return out


# -- the whole pipeline ----------------------------------------------------

def plan(mode: str, X: Mat, Z: Mat, camera: CameraModel, board: BoardSpec, seed_tool: Mat,
         start_joints: Mat | None, captured: list[Mat], ik: IKFunction, cfg: SessionConfig,
         solver_cfg: SolverConfig, seed: int = 0, uncertainty_px: float | None = None) -> Plan:
    """§C1-8. ``captured``: tool poses already captured (bootstrap), counted in
    the selection and the information check.

    The plan's ``target`` is the number of informative poses it relies on: at
    least ``target_captures``, more if the information check needed more. The
    spares are a reserve on top (target x (reserve_factor - 1)). If the
    candidates run out before the information clears, the plan is weak.
    """
    rejections: Counter[str] = Counter()
    ok: list[Candidate] = []
    for C in sample(camera, board, cfg, seed):
        c = Candidate(C)
        c.reject, c.detail = visibility(C, camera, board, cfg, uncertainty_px)
        if c.reject is None:
            c.tool_pose = handeye.tool_pose_for(mode, X, Z, C)
            c.reject, c.detail = limits(c.tool_pose, seed_tool, cfg,
                                        moving_points_base(mode, c.tool_pose, X, Z, board))
        if c.reject is None:
            res = ik(c.tool_pose)
            if res.joints is None:
                c.reject, c.detail = IK_FAILED, res.reason
            else:
                c.joints = np.asarray(res.joints, dtype=float)
        if c.reject is None:
            ok.append(c)
        else:
            rejections[c.reject] += 1

    # best first; enough for the target, the information top-up and the reserve
    ranked = select(mode, ok, captured, cfg,
                    count=min(len(ok), int(np.ceil(3 * cfg.target_captures * cfg.reserve_factor))))
    main = ranked[:cfg.target_captures]
    rest = ranked[cfg.target_captures:]

    def info() -> float:
        return information(mode, X, Z, captured + [c.tool_pose for c in main], camera, board,
                           solver_cfg)
    current = info()
    while current < cfg.min_plan_singular_value and rest:
        main.append(rest.pop(0))
        current = info()
    n_spares = int(np.ceil(len(main) * (cfg.reserve_factor - 1.0)))
    spares = rest[:n_spares]
    poses = order(main, start_joints) + order(spares, main[-1].joints if main else None)
    return Plan(poses, len(main), current, current < cfg.min_plan_singular_value, rejections)
