"""Calibration targets: ChArUco and parity-asymmetric checkerboards (02-features §J).

Board frame (both kinds): origin at the top-left outer corner of the printed
square grid, x to the right, y down the printout, z into the board (away from a
viewer facing the print). Square (col i, row j) is black when i + j is even.

ChArUco corner IDs and positions come from OpenCV. Checkerboard corner IDs are
``row * cols + col`` over the inner corners, with inner corner (c, r) at
``((c + 1) * square, (r + 1) * square, 0)``.

YAML keeps square counts (``squares_x``/``squares_y``) and never inner-corner
counts, so the two can't be confused.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from functools import cached_property
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import cv_compat

KINDS = ('charuco', 'checkerboard')


class BoardSpecError(ValueError):
    pass


@dataclass(frozen=True)
class BoardSpec:
    kind: str
    squares_x: int
    squares_y: int
    square_length: float          # m, measured on the print
    marker_length: float = 0.0    # m, ChArUco only
    dictionary: str = ''          # ChArUco only, e.g. 'DICT_5X5_100'

    def __post_init__(self) -> None:
        if self.kind not in KINDS:
            raise BoardSpecError(f'board kind must be one of {KINDS}, got {self.kind!r}')
        if self.squares_x < 3 or self.squares_y < 3:
            raise BoardSpecError('a board needs at least 3 squares per side')
        if self.square_length <= 0:
            raise BoardSpecError('square_length must be positive (metres)')
        if self.kind == 'checkerboard':
            if min(self.squares_x, self.squares_y) < 4:
                # findChessboardCornersSB fails on a 2-corner side (seen on 4.6).
                raise BoardSpecError('a checkerboard needs at least 4 squares per side')
            if (self.squares_x % 2) == (self.squares_y % 2):
                raise BoardSpecError(
                    f'checkerboard {self.squares_x}x{self.squares_y} squares is symmetric '
                    '(its 180-degree rotation looks the same), so its orientation is '
                    'ambiguous. Use one odd and one even square count, or a ChArUco board.')
        else:
            if not 0 < self.marker_length < self.square_length:
                raise BoardSpecError('marker_length must be in (0, square_length)')
            dictionary = cv_compat.get_dictionary(self.dictionary)  # raises if unknown
            n_markers = (self.squares_x * self.squares_y) // 2
            if n_markers > cv_compat.dictionary_size(dictionary):
                raise BoardSpecError(
                    f'{self.dictionary} has too few markers for a '
                    f'{self.squares_x}x{self.squares_y} ChArUco board')

    # -- geometry ---------------------------------------------------------
    @property
    def inner_corners(self) -> tuple[int, int]:
        """(cols, rows) of inner corners."""
        return (self.squares_x - 1, self.squares_y - 1)

    @property
    def size_m(self) -> tuple[float, float]:
        return (self.squares_x * self.square_length, self.squares_y * self.square_length)

    @cached_property
    def opencv_board(self) -> Any:
        if self.kind != 'charuco':
            raise BoardSpecError('only ChArUco boards have an OpenCV board object')
        return cv_compat.make_charuco_board(
            self.squares_x, self.squares_y, self.square_length, self.marker_length,
            cv_compat.get_dictionary(self.dictionary))

    @cached_property
    def corner_points(self) -> NDArray[np.float64]:
        """(N, 3) corner positions; row index = corner ID."""
        if self.kind == 'charuco':
            return cv_compat.chessboard_corners(self.opencv_board)
        cols, rows = self.inner_corners
        c, r = np.meshgrid(np.arange(cols), np.arange(rows))
        pts = np.stack([(c.ravel() + 1), (r.ravel() + 1), np.zeros(c.size)], axis=1)
        scale = np.array([self.square_length, self.square_length, 1.0])
        return np.asarray(pts * scale, dtype=np.float64)

    @property
    def outline(self) -> NDArray[np.float64]:
        """(4, 3) outer corners of the square grid."""
        w, h = self.size_m
        return np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], dtype=float)

    # -- serialisation ----------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        if self.kind == 'checkerboard':
            d.pop('marker_length')
            d.pop('dictionary')
        return d

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> BoardSpec:
        if 'inner_corners_x' in d or 'inner_corners_y' in d:
            raise BoardSpecError(
                'give square counts (squares_x, squares_y), not inner-corner counts')
        return cls(**d)


# -- printable images -----------------------------------------------------

def board_texture(spec: BoardSpec, px_per_square: int, margin_px: int = 0
                  ) -> tuple[NDArray[np.uint8], NDArray[np.float64]]:
    """Board image and the 3x3 map from board (X, Y, 1) in metres to texture pixels.

    Texture coordinates follow cv2.remap: integer values are pixel centres.
    """
    w = spec.squares_x * px_per_square + 2 * margin_px
    h = spec.squares_y * px_per_square + 2 * margin_px
    if spec.kind == 'charuco':
        img = cv_compat.board_image(spec.opencv_board, (w, h), margin_px)
    else:
        img = _checkerboard_image(spec, px_per_square, margin_px)
    s = px_per_square / spec.square_length
    tex_from_board = np.array([[s, 0, margin_px - 0.5],
                               [0, s, margin_px - 0.5],
                               [0, 0, 1.0]])
    return img, tex_from_board


def _checkerboard_image(spec: BoardSpec, pps: int, margin: int) -> NDArray[np.uint8]:
    j, i = np.mgrid[0:spec.squares_y, 0:spec.squares_x]
    squares = np.where((i + j) % 2 == 0, 0, 255).astype(np.uint8)
    img = np.kron(squares, np.ones((pps, pps), dtype=np.uint8))
    return np.pad(img, margin, constant_values=255)
