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
    advice: str = ''                       # what to change when weak (shown before execution)
    ik_errors: Counter[str] = field(default_factory=Counter)   # IK failure detail (MoveIt code)


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
    """Smallest projected marker cell (px) over the whole board.

    Each marker is a square of ``bits + 2`` cells per side (inner bits from the
    OpenCV dictionary plus the black border). The cell grid is built in the
    board frame (bilinear in the marker corners, exact for a planar square),
    every grid point is projected with the full camera model, and the
    shortest cell edge anywhere wins, so foreshortened cells count.
    """
    from . import cv_compat
    n = cv_compat.marker_bits(cv_compat.get_dictionary(board.dictionary)) + 2
    _, obj = cv_compat.marker_object_points(board.opencv_board)            # (M, 4, 3)
    s = np.linspace(0.0, 1.0, n + 1)
    S, T = np.meshgrid(s, s, indexing='xy')                                # (n+1, n+1)
    w = np.stack([(1 - S) * (1 - T), S * (1 - T), S * T, (1 - S) * T], axis=-1)
    grid = np.einsum('rck,mkd->mrcd', w, obj)                              # (M, n+1, n+1, 3)
    px = camera.project_object(C, grid.reshape(-1, 3)).reshape(grid.shape[:3] + (2,))
    along_r = np.linalg.norm(np.diff(px, axis=1), axis=-1)
    along_c = np.linalg.norm(np.diff(px, axis=2), axis=-1)
    return float(min(along_r.min(), along_c.min()))


# -- 4. robot-agnostic limits ----------------------------------------------

def box_contains(points_base: Mat, cfg: SessionConfig, margin: float = 0.0) -> bool:
    """Are all points (base frame) inside the posed safety box, at least
    ``margin`` (m) from every face?"""
    B = tf.make(Rotation.from_euler('xyz', cfg.box_rpy), np.asarray(cfg.box_xyz, dtype=float))
    local = tf.apply(tf.inv(B), np.atleast_2d(points_base))
    half = np.asarray(cfg.box_size) / 2 - margin
    return bool(np.all(np.abs(local) <= half))


def limits(G: Mat, seed_tool: Mat, cfg: SessionConfig,
           moving_points: Mat | None = None, box_margin: float = 0.0,
           orientation_margin: float = 0.0) -> tuple[str | None, str]:
    """(margins: a point between two checked waypoints can bulge out of the box
    by the arc sag, and its orientation from the seed exceed the endpoints' by
    up to half the segment angle; the caller passes both.)"""
    """Safety box (toggle), envelope around the seed, orientation limit from the
    seed. The envelope and orientation limits always apply; the box adds a
    user-placed volume on top (01-architecture §3.4)."""
    p = G[:3, 3]
    if cfg.box_enabled:
        pts = p[None] if moving_points is None or not cfg.box_check_moving_object else \
            np.vstack([p[None], moving_points])
        if not box_contains(pts, cfg, box_margin):
            return OUTSIDE_BOX, 'outside the safety box'
    d = float(np.linalg.norm(p - seed_tool[:3, 3]))
    if d > cfg.envelope_radius:
        return OUTSIDE_ENVELOPE, f'{d:.2f} m from the seed > {cfg.envelope_radius}'
    a = tf.rotation_angle(G, seed_tool)
    if a > cfg.max_orientation_change - orientation_margin:
        return ORIENTATION_LIMIT, f'{np.degrees(a):.0f} deg from the seed orientation'
    return None, ''


def lever_ball_points(mode: str, G: Mat, cfg: SessionConfig, board: BoardSpec) -> Mat:
    """Before any estimate: where the carried object can be. Its reference point
    (board centre for eye_to_hand, camera origin for eye_in_hand) is within
    sqrt(3) L_max of the tool origin; for eye_to_hand every board corner is
    within that plus the board's half-diagonal. The six extremes of that ball
    along the box axes lie inside the (convex, axis-aligned in its own frame)
    box iff the whole ball does."""
    B = Rotation.from_euler('xyz', cfg.box_rpy).as_matrix()
    r = np.sqrt(3) * cfg.lever_max
    if mode == 'eye_to_hand':
        r += 0.5 * float(np.hypot(*board.size_m))
    offsets = np.vstack([B.T * r, -B.T * r])               # rows: +-r along each box axis
    return np.asarray(G[:3, 3] + offsets, dtype=np.float64)


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
    """Angles (...,) and unit axes (..., 3) of Ra^T Rb, batched over leading dims
    (SciPy rotation vectors: robust near 0 and near a half-turn)."""
    rel = np.einsum('...ji,...jk->...ik', Ra, Rb)
    shape = rel.shape[:-2]
    rv = Rotation.from_matrix(rel.reshape(-1, 3, 3)).as_rotvec().reshape(shape + (3,))
    ang = np.linalg.norm(rv, axis=-1)
    axes = np.where(ang[..., None] > 0, rv / np.maximum(ang, 1e-300)[..., None], 0.0)
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
    ik_errors: Counter[str] = Counter()
    ok: list[Candidate] = []
    for C in sample(camera, board, cfg, seed):
        c = Candidate(C)
        c.reject, c.detail = visibility(C, camera, board, cfg, uncertainty_px)
        if c.reject is None:
            c.tool_pose = handeye.tool_pose_for(mode, X, Z, C)
            # the carried object comes from the estimate: same margin as execution
            c.reject, c.detail = limits(c.tool_pose, seed_tool, cfg,
                                        moving_points_base(mode, c.tool_pose, X, Z, board),
                                        cfg.estimate_box_margin
                                        if cfg.box_check_moving_object else 0.0)
        if c.reject is None:
            res = ik(c.tool_pose)
            if res.joints is None:
                c.reject, c.detail = IK_FAILED, res.reason
                ik_errors[res.reason or 'unknown'] += 1
            else:
                c.joints = np.asarray(res.joints, dtype=float)
        if c.reject is None:
            ok.append(c)
        else:
            rejections[c.reject] += 1

    # Budget: the target may grow by information top-up up to
    # target_captures x reserve_factor (max_target); spares are
    # target x (reserve_factor - 1) on top, all within
    # budget = ceil(target_captures x reserve_factor^2) moves; if max_target
    # poses are not informative enough, the plan is weak (reported before execution).
    max_target = int(np.ceil(cfg.target_captures * cfg.reserve_factor))
    budget = int(np.ceil(cfg.target_captures * cfg.reserve_factor ** 2))
    ranked = select(mode, ok, captured, cfg, count=min(len(ok), budget))
    main = ranked[:cfg.target_captures]
    rest = ranked[cfg.target_captures:]

    def info() -> float:
        return information(mode, X, Z, captured + [c.tool_pose for c in main], camera, board,
                           solver_cfg)
    current = info()
    while current < cfg.min_plan_singular_value and rest and len(main) < max_target:
        main.append(rest.pop(0))
        current = info()
    n_spares = min(int(np.ceil(len(main) * (cfg.reserve_factor - 1.0))), budget - len(main))
    spares = rest[:n_spares]
    poses = order(main, start_joints) + order(spares, main[-1].joints if main else None)
    # weak: not informative enough, or fewer usable poses than the configured
    # target (prior captures alone must not make an empty plan look strong)
    weak = current < cfg.min_plan_singular_value or len(main) < cfg.target_captures
    return Plan(poses, len(main), current, weak, rejections,
                weak_advice(rejections, len(ok), cfg.target_captures, current,
                            cfg.min_plan_singular_value) if weak else '', ik_errors)


def weak_advice(rejections: Counter[str], usable: int, target: int = 0,
                information: float = 0.0, threshold: float = 0.0) -> str:
    """What to change when a plan is weak, from what rejected the candidates."""
    tips = {OUTSIDE_BOX: 'enlarge or move the safety box',
            OUTSIDE_ENVELOPE: 'widen the envelope radius',
            ORIENTATION_LIMIT: 'raise the orientation limit',
            IK_FAILED: 'move the seed to a more dexterous part of the workspace',
            NOT_VISIBLE: 'use a wider lens or a smaller board, or increase the depth range',
            COVERAGE: 'adjust the depth range or the coverage band',
            TOO_SMALL: 'use a bigger board or bring it closer',
            BACK_FACING: 'reduce the maximum tilt'}
    top = [r for r, _ in rejections.most_common() if r in tips][:2]
    text = f'plan is weak: {usable} usable candidate poses'
    if usable < target:
        text += f' for a target of {target}'
    text += f'; rotation information {information:.0f} (needs {threshold:.0f})'
    if top:
        text += '; most were rejected as ' + ', '.join(
            f'{r} ({rejections[r]})' for r in top) + ': ' + '; '.join(tips[r] for r in top)
    return text
