"""Board detection with stray-marker masking (02-features §D, §D4).

Reuse: ``findChessboardCornersSB`` (already subpixel), OpenCV ArUco/ChArUco
via ``cv_compat``, ``solvePnPRansac``, ``cornerSubPix``. Written here: the glue
for masking (duplicate IDs, RANSAC inliers, support checks, two-board check),
a canonical corner order for checkerboards, and a final ``cornerSubPix`` on
ChArUco corners (OpenCV's interpolation returns them ~0.5 px off on 4.6 and
4.10; measured in test_detection.py).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np
from numpy.typing import NDArray

from . import codes, cv_compat
from .boards import BoardSpec
from .camera import CameraModel
from .config import CaptureConfig

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)


@dataclass
class Detection:
    ids: NDArray[np.int32]                  # ascending
    image_points: NDArray[np.float64]       # (N, 2) px, same order
    object_points: NDArray[np.float64]      # (N, 3) m, board frame
    reject: str | None = None
    detail: str = ''
    info: dict[str, float] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.reject is None

    @staticmethod
    def rejected(code: str, detail: str, **info: float) -> Detection:
        return Detection(np.zeros(0, np.int32), np.zeros((0, 2)), np.zeros((0, 3)),
                         code, detail, dict(info))


def detect(gray: NDArray[np.uint8], spec: BoardSpec, camera: CameraModel,
           cfg: CaptureConfig) -> Detection:
    if gray.ndim != 2:
        raise ValueError('detect() expects a single-channel image')
    if spec.kind == 'checkerboard':
        return _detect_checkerboard(gray, spec, camera, cfg)
    return _detect_charuco(gray, spec, camera, cfg)


# -- checkerboard ---------------------------------------------------------

def _detect_checkerboard(gray: NDArray[np.uint8], spec: BoardSpec, camera: CameraModel,
                         cfg: CaptureConfig) -> Detection:
    cols, rows = spec.inner_corners
    found, corners = cv2.findChessboardCornersSB(
        gray, (cols, rows), flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        return Detection.rejected(codes.NO_BOARD, 'checkerboard not found')
    px = canonical_checkerboard_order(
        np.asarray(corners, dtype=np.float64).reshape(rows, cols, 2), gray).reshape(-1, 2)
    ids = np.arange(cols * rows, dtype=np.int32)
    agree = square_pattern_agreement(gray, px, spec, camera)
    if agree < cfg.min_pattern_agreement:
        # Seen in the M1a pose sweep: the detector can take the board edge against
        # the white margin as a row of corners, giving a grid turned by 90 degrees.
        return Detection.rejected(codes.NO_BOARD, f'checkerboard grid does not match the '
                                  f'board ({agree:.0%} of squares agree)',
                                  pattern_agreement=agree)
    return Detection(ids, px, spec.corner_points.copy(), info={'pattern_agreement': agree})


def square_pattern_agreement(gray: NDArray[np.uint8], px: NDArray[np.float64],
                             spec: BoardSpec, camera: CameraModel) -> float:
    """Fraction of ALL squares (outer ring included) whose colour matches the board.

    The plane-to-image map comes from the detected corners (a homography in
    undistorted coordinates, re-distorted with the camera model), so a grid
    that is shifted or turned by one square disagrees on the outer ring.
    """
    norm = camera.normalized(px)
    H, _ = cv2.findHomography(spec.corner_points[:, :2], norm)
    if H is None:
        return 0.0
    j, i = np.mgrid[0:spec.squares_y, 0:spec.squares_x]
    centres = np.stack([(i.ravel() + 0.5), (j.ravel() + 0.5)], 1) * spec.square_length
    xy = cv2.perspectiveTransform(centres.reshape(-1, 1, 2), H).reshape(-1, 2)
    img_px = camera.project(np.c_[xy, np.ones(len(xy))])
    h, w = gray.shape
    inside = (img_px[:, 0] >= 1) & (img_px[:, 0] < w - 2) & \
        (img_px[:, 1] >= 1) & (img_px[:, 1] < h - 2)
    if inside.sum() < 0.5 * len(img_px):
        return 0.0
    vals = cv2.remap(gray.astype(np.float32), img_px[:, 0].astype(np.float32).reshape(1, -1),
                     img_px[:, 1].astype(np.float32).reshape(1, -1),
                     cv2.INTER_LINEAR).ravel()[inside]
    black = ((i.ravel() + j.ravel()) % 2 == 0)[inside]
    if black.all() or not black.any():
        return 0.0
    threshold = 0.5 * (np.median(vals[black]) + np.median(vals[~black]))
    return float(np.mean((vals < threshold) == black))


def canonical_checkerboard_order(grid: NDArray[np.float64], gray: NDArray[np.uint8]
                                 ) -> NDArray[np.float64]:
    """Reorder a (rows, cols, 2) corner grid so corner 0 matches the board frame.

    The detector may start at any grid corner. Two checks fix it:
    1. Handedness: board x (along a row) crossed with board y (down a column)
       must point into the image (positive in pixel coordinates, where y is
       down) for a board seen from its printed side. Otherwise mirror the rows.
    2. Colour: the square between inner corners (0,0) and (1,1) is board
       square (1,1), which is black. For a parity-asymmetric board its 180-
       degree counterpart is white, so if it's lighter than its neighbour
       square, rotate the grid by 180 degrees.
    """
    g = grid
    x_dir = g[0, -1] - g[0, 0]
    y_dir = g[-1, 0] - g[0, 0]
    if x_dir[0] * y_dir[1] - x_dir[1] * y_dir[0] < 0:
        g = g[:, ::-1]
    first = _cell_mean(gray, g, 0, 0)        # board square (1, 1): black
    neighbour = _cell_mean(gray, g, 0, 1)    # board square (2, 1): white
    if first > neighbour:
        g = g[::-1, ::-1]
    return g


def _cell_mean(gray: NDArray[np.uint8], g: NDArray[np.float64], r: int, c: int) -> float:
    quad = np.array([g[r, c], g[r, c + 1], g[r + 1, c + 1], g[r + 1, c]])
    centre = quad.mean(axis=0)
    shrunk = (centre + 0.5 * (quad - centre)).astype(np.int32)  # stay inside the square
    mask = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(mask, shrunk, 1)
    return float(gray[mask.astype(bool)].mean()) if mask.any() else float(
        gray[int(centre[1]), int(centre[0])])


# -- ChArUco with stray-marker masking ------------------------------------

def _detect_charuco(gray: NDArray[np.uint8], spec: BoardSpec, camera: CameraModel,
                    cfg: CaptureConfig) -> Detection:
    board = spec.opencv_board
    dictionary = cv_compat.get_dictionary(spec.dictionary)
    m_corners, m_ids = cv_compat.detect_markers(gray, dictionary)
    board_ids, board_obj = cv_compat.marker_object_points(board)
    on_board = np.isin(m_ids, board_ids)
    m_corners, m_ids = m_corners[on_board], m_ids[on_board]
    if len(m_ids) == 0:
        return Detection.rejected(codes.NO_BOARD, 'no board markers detected')
    info: dict[str, float] = {'markers_detected': float(len(m_ids))}

    if cfg.masking:
        keep, reject = mask_markers(m_corners, m_ids, board_ids, board_obj, spec, camera,
                                    cfg, info)
        if reject is not None:
            return Detection.rejected(reject[0], reject[1], **info)
        m_corners, m_ids = m_corners[keep], m_ids[keep]

    ids, px = cv_compat.interpolate_charuco(gray, board, m_corners, m_ids)
    if len(ids) == 0:
        return Detection.rejected(codes.FEW_FEATURES, 'no ChArUco corners interpolated',
                                  **info)
    order = np.argsort(ids)
    ids, px, kept = _refine_corners(gray, ids[order], px[order], m_corners, spec, cfg)
    info['corners_dropped_small'] = float(kept.size - kept.sum())
    ids, px = ids[kept], px[kept]
    if len(ids) == 0:
        return Detection.rejected(codes.FEW_FEATURES, 'all corners too small to refine',
                                  **info)
    return Detection(ids, px, spec.corner_points[ids].copy(), info=info)


def _refine_corners(gray: NDArray[np.uint8], ids: NDArray[np.int32],
                    px: NDArray[np.float64], marker_corners: NDArray[np.float64],
                    spec: BoardSpec, cfg: CaptureConfig
                    ) -> tuple[NDArray[np.int32], NDArray[np.float64], NDArray[np.bool_]]:
    """cornerSubPix per corner, with a window inside the local white marker border.

    OpenCV's interpolated ChArUco corners are ~0.5 px off (4.6, 4.10, 4.12), so
    every corner is refined. A window reaching into the marker bits biases the
    corner by 0.2-0.3 px; ~0.8 of the border width gives ~0.04 px (measured on
    rendered boards). The border is estimated from the shortest edge of the two
    nearest markers, so foreshortened regions get a smaller window.
    shortcut: corners whose safe window is under 2 px are dropped rather than
    refined; revisit if small/far boards lose too many corners on real images.
    """
    centres = marker_corners.mean(axis=1)
    edges = np.linalg.norm(marker_corners - np.roll(marker_corners, 1, axis=1), axis=2)
    shortest = edges.min(axis=1)
    ratio = (spec.square_length - spec.marker_length) / (2 * spec.marker_length)
    out = px.copy()
    kept = np.zeros(len(ids), dtype=bool)
    for k, p in enumerate(px):
        nearest = np.argsort(np.linalg.norm(centres - p, axis=1))[:2]
        half = int(min(round(0.8 * float(shortest[nearest].min()) * ratio),
                       cfg.subpix_max_half_window))
        if half < 2:
            continue
        pt = p.astype(np.float32).reshape(1, 1, 2)
        out[k] = cv2.cornerSubPix(gray, pt, (half, half), (-1, -1),
                                  SUBPIX_CRITERIA).reshape(2)
        kept[k] = True
    return ids, out, kept


def mask_markers(m_corners: NDArray[np.float64], m_ids: NDArray[np.int32],
                 board_ids: NDArray[np.int32], board_obj: NDArray[np.float64],
                 spec: BoardSpec, camera: CameraModel, cfg: CaptureConfig,
                 info: dict[str, float]) -> tuple[NDArray[np.bool_], tuple[str, str] | None]:
    """Stray-marker masking, 02-features §D4 steps 2-4.

    Returns a keep-mask over the detected markers, or a rejection (code, detail).
    TWO_BOARDS is only returned on geometric evidence (a second consistent fit).
    """
    obj_of = {int(i): board_obj[k] for k, i in enumerate(board_ids)}
    uniq, counts = np.unique(m_ids, return_counts=True)
    dup_ids = set(uniq[counts > 1].tolist())
    unique_idx = np.flatnonzero([int(i) not in dup_ids for i in m_ids])
    info['duplicate_markers'] = float(len(m_ids) - len(unique_idx))
    nothing = np.zeros(len(m_ids), dtype=bool)

    # Fit on markers with a unique ID; if too few, on all copies (each copy is a
    # separate hypothesis and RANSAC treats the wrong copies as outliers).
    fit_idx = unique_idx if len(unique_idx) >= cfg.min_markers else np.arange(len(m_ids))
    info['fit_used_duplicates'] = float(len(fit_idx) != len(unique_idx))
    if len(fit_idx) < cfg.min_markers:
        return nothing, (codes.MARKER_SUPPORT,
                         f'{len(fit_idx)} board markers detected < {cfg.min_markers}')
    pose = _ransac(m_corners[fit_idx], [obj_of[int(i)] for i in m_ids[fit_idx]], camera, cfg)
    if pose is None:
        return nothing, (codes.MARKER_SUPPORT, 'RANSAC found no consistent board pose')

    # A marker is an inlier only if all four corners reproject within the threshold;
    # of several inlier copies of one ID, keep the best.
    err = np.array([_marker_error(m_corners[k], obj_of[int(m_ids[k])], *pose, camera)
                    for k in range(len(m_ids))])
    fits = err <= cfg.ransac_reprojection_px     # geometrically consistent with pose 1
    inlier = fits.copy()
    for d in dup_ids:
        copies = np.flatnonzero((m_ids == d) & inlier)
        if len(copies) > 1:
            inlier[copies] = False
            inlier[copies[np.argmin(err[copies])]] = True
    n_in = int(inlier.sum())                       # one copy per ID, so distinct IDs
    info['inlier_markers'] = float(n_in)
    info['outlier_markers'] = float(len(m_ids) - n_in)

    if n_in < cfg.min_markers:
        return inlier, (codes.MARKER_SUPPORT, f'{n_in} inlier markers < {cfg.min_markers}')
    hull = hull_fraction(np.concatenate([obj_of[int(i)] for i in m_ids[inlier]]), spec)
    info['inlier_marker_hull_fraction'] = hull
    if hull < cfg.min_marker_hull_fraction:
        return inlier, (codes.MARKER_SUPPORT,
                        f'inlier markers cover {hull:.2f} of the board '
                        f'< {cfg.min_marker_hull_fraction}')

    # Two boards: a second consistent group of comparable size among the GEOMETRIC
    # outliers (extra copies that fit pose 1 are repeats of the same board, not evidence).
    out_idx = np.flatnonzero(~fits)
    need = max(cfg.two_boards_min_markers, int(np.ceil(cfg.two_boards_ratio * n_in)))
    info['second_group_markers'] = 0.0
    if len(np.unique(m_ids[out_idx])) >= need:
        second = _ransac(m_corners[out_idx], [obj_of[int(i)] for i in m_ids[out_idx]],
                         camera, cfg)
        if second is not None:
            ok = [k for k in out_idx if _marker_error(
                m_corners[k], obj_of[int(m_ids[k])], *second, camera)
                <= cfg.ransac_reprojection_px]
            group = len(np.unique(m_ids[ok]))      # distinct IDs only
            info['second_group_markers'] = float(group)
            if group >= need:
                return inlier, (codes.TWO_BOARDS,
                                f'a second consistent group of {group} markers')
    return inlier, None


def _ransac(corners: NDArray[np.float64], objs: list[NDArray[np.float64]],
            camera: CameraModel, cfg: CaptureConfig
            ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
    img = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    obj = np.concatenate(objs).reshape(-1, 3)
    if len(obj) < 4:
        return None
    cv2.setRNGSeed(0)   # deterministic RANSAC
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        obj, img, camera.K, camera.D, iterationsCount=500,
        reprojectionError=cfg.ransac_reprojection_px, confidence=0.999,
        flags=cv2.SOLVEPNP_IPPE)
    if not ok or inl is None or len(inl) < 4:
        return None
    return (np.asarray(rvec, dtype=np.float64).reshape(3),
            np.asarray(tvec, dtype=np.float64).reshape(3))


def _marker_error(corners: NDArray[np.float64], obj: NDArray[np.float64],
                  rvec: NDArray[np.float64], tvec: NDArray[np.float64],
                  camera: CameraModel) -> float:
    proj, _ = cv2.projectPoints(obj, rvec, tvec, camera.K, camera.D)
    return float(np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1).max())


def hull_fraction(object_points: NDArray[np.float64], spec: BoardSpec) -> float:
    """Convex-hull area of board points as a fraction of the board area."""
    pts = np.asarray(object_points[:, :2], dtype=np.float32)
    if len(pts) < 3:
        return 0.0
    area = cv2.contourArea(cv2.convexHull(pts))
    w, h = spec.size_m
    return float(area / (w * h))
