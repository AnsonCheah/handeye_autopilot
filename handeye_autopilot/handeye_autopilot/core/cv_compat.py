"""OpenCV ArUco/ChArUco compatibility shim (reuse map: board detection).

OpenCV 4.6 (Ubuntu 24.04, Jazzy) has the old function API
(``CharucoBoard_create``, ``detectMarkers``, ``interpolateCornersCharuco``);
4.7+ (4.10 on 26.04, Lyrical) has the class API (``CharucoBoard``,
``ArucoDetector``, ``CharucoDetector``). Everything else in the core calls
these functions only.

Board layout: Ubuntu's 4.6 already draws the same layout as 4.7+ with
``legacyPattern = False`` (checked for odd and even row counts); fixture tests
on both versions guard this.
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

aruco: Any = cv2.aruco   # typed as Any: the 4.6 and 4.10 APIs differ
NEW_API = hasattr(aruco, 'ArucoDetector')


def get_dictionary(name: str) -> Any:
    key = name if name.startswith('DICT_') else 'DICT_' + name
    if not hasattr(aruco, key):
        raise ValueError(f'unknown ArUco dictionary {name!r}')
    return aruco.getPredefinedDictionary(getattr(aruco, key))


def dictionary_size(dictionary: Any) -> int:
    return int(np.asarray(dictionary.bytesList).shape[0])


def make_charuco_board(squares_x: int, squares_y: int, square_length: float,
                       marker_length: float, dictionary: Any) -> Any:
    if NEW_API:
        board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length,
                                   dictionary)
        board.setLegacyPattern(False)
        return board
    return aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length,
                                     dictionary)


def chessboard_corners(board: Any) -> NDArray[np.float64]:
    """(N, 3) ChArUco corner positions in the board frame; index = corner ID."""
    pts = board.getChessboardCorners() if NEW_API else board.chessboardCorners
    return np.asarray(pts, dtype=np.float64).reshape(-1, 3)


def marker_object_points(board: Any) -> tuple[NDArray[np.int32], NDArray[np.float64]]:
    """Marker IDs (M,) and their corners (M, 4, 3) in the board frame."""
    if NEW_API:
        ids, pts = board.getIds(), board.getObjPoints()
    else:
        ids, pts = board.ids, board.objPoints
    return (np.asarray(ids, dtype=np.int32).reshape(-1),
            np.asarray(pts, dtype=np.float64).reshape(-1, 4, 3))


def board_image(board: Any, size: tuple[int, int], margin: int,
                border_bits: int = 1) -> NDArray[np.uint8]:
    if NEW_API:
        img = board.generateImage(size, marginSize=margin, borderBits=border_bits)
    else:
        img = board.draw(size, marginSize=margin, borderBits=border_bits)
    return np.asarray(img, dtype=np.uint8)


def marker_image(dictionary: Any, marker_id: int, side_px: int) -> NDArray[np.uint8]:
    if NEW_API:
        img = aruco.generateImageMarker(dictionary, marker_id, side_px)
    else:
        img = aruco.drawMarker(dictionary, marker_id, side_px)
    return np.asarray(img, dtype=np.uint8)


def detect_markers(gray: NDArray[np.uint8], dictionary: Any
                   ) -> tuple[NDArray[np.float64], NDArray[np.int32]]:
    """Detect markers only. Returns corners (M, 4, 2) and IDs (M,)."""
    if NEW_API:
        detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        params = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None or len(ids) == 0:
        return np.zeros((0, 4, 2)), np.zeros(0, dtype=np.int32)
    return (np.asarray(corners, dtype=np.float64).reshape(-1, 4, 2),
            np.asarray(ids, dtype=np.int32).reshape(-1))


def interpolate_charuco(gray: NDArray[np.uint8], board: Any,
                        marker_corners: NDArray[np.float64],
                        marker_ids: NDArray[np.int32]
                        ) -> tuple[NDArray[np.int32], NDArray[np.float64]]:
    """ChArUco corners from the given markers only (no re-detection).

    Returns IDs (N,) and pixels (N, 2), unsorted as OpenCV gives them.
    """
    if len(marker_ids) == 0:
        return np.zeros(0, dtype=np.int32), np.zeros((0, 2))
    corners = [c.reshape(1, 4, 2).astype(np.float32) for c in marker_corners]
    ids = np.asarray(marker_ids, dtype=np.int32).reshape(-1, 1)
    if NEW_API:
        params = aruco.CharucoParameters()
        # Our own masking (detection.py) decides which markers are trusted; turn
        # off the 4.7+ extras so 4.6 and 4.10/4.12 interpolate the same way.
        for name in ('checkMarkers', 'tryRefineMarkers'):
            if hasattr(params, name):
                setattr(params, name, False)
        detector = aruco.CharucoDetector(board, charucoParams=params)
        ch_corners, ch_ids, _, _ = detector.detectBoard(
            gray, markerCorners=corners, markerIds=ids)
    else:
        _, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if ch_ids is None or len(ch_ids) == 0:
        return np.zeros(0, dtype=np.int32), np.zeros((0, 2))
    return (np.asarray(ch_ids, dtype=np.int32).reshape(-1),
            np.asarray(ch_corners, dtype=np.float64).reshape(-1, 2))
