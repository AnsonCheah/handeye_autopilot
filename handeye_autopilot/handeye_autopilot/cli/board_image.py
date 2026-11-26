"""Write a printable board image at an exact physical scale (02-features §J).

Example:
  ros2 run handeye_autopilot board_image charuco 7 5 --square 0.04 --marker 0.03 \
      --dictionary DICT_5X5_100 --dpi 300 -o board.png
Print at 100 % scale (no "fit to page"), then measure a square with calipers
and put the measured value in the board config.
"""

from __future__ import annotations

import argparse
import struct
import sys
import zlib

import cv2

from handeye_autopilot.core.boards import BoardSpec, BoardSpecError, board_texture

INCH = 0.0254


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('kind', choices=['charuco', 'checkerboard'])
    p.add_argument('squares_x', type=int, help='number of squares (not inner corners)')
    p.add_argument('squares_y', type=int)
    p.add_argument('--square', type=float, required=True, help='square side, m')
    p.add_argument('--marker', type=float, default=0.0, help='marker side, m (ChArUco)')
    p.add_argument('--dictionary', default='', help='e.g. DICT_5X5_100 (ChArUco)')
    p.add_argument('--dpi', type=int, default=300)
    p.add_argument('--margin', type=float, default=0.01, help='white margin, m')
    p.add_argument('-o', '--output', required=True)
    a = p.parse_args(argv)
    if a.dpi <= 0 or a.margin < 0 or a.square <= 0:
        print('error: --dpi and --square must be positive and --margin non-negative',
              file=sys.stderr)
        return 2
    if round(a.square / INCH * a.dpi) < 10:
        print(f'error: {a.square} m at {a.dpi} dpi is under 10 px per square; raise --dpi',
              file=sys.stderr)
        return 2
    try:
        spec = BoardSpec(a.kind, a.squares_x, a.squares_y, a.square, a.marker, a.dictionary)
    except BoardSpecError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    pps_exact = a.square / INCH * a.dpi
    pps = round(pps_exact)
    if abs(pps - pps_exact) / pps_exact > 0.001:
        print(f'warning: {a.square} m at {a.dpi} dpi is {pps_exact:.2f} px per square; '
              f'rounded to {pps}, so the printed square is '
              f'{pps / a.dpi * INCH * 1000:.3f} mm. Measure it.', file=sys.stderr)
    img, _ = board_texture(spec, pps, round(a.margin / INCH * a.dpi))
    ok, png = cv2.imencode('.png', img)
    if not ok:
        print('error: PNG encoding failed', file=sys.stderr)
        return 1
    try:
        with open(a.output, 'wb') as f:
            f.write(with_dpi(png.tobytes(), a.dpi))
    except OSError as e:
        print(f'error: could not write {a.output}: {e}', file=sys.stderr)
        return 1
    w_mm, h_mm = (img.shape[1] / a.dpi * 25.4, img.shape[0] / a.dpi * 25.4)
    print(f'wrote {a.output}: {img.shape[1]}x{img.shape[0]} px = {w_mm:.1f}x{h_mm:.1f} mm '
          f'at {a.dpi} dpi')
    return 0


def with_dpi(png: bytes, dpi: int) -> bytes:
    """Insert a pHYs chunk (pixels per metre) after IHDR so viewers print to scale."""
    ppm = round(dpi / INCH)
    body = b'pHYs' + struct.pack('>IIB', ppm, ppm, 1)
    chunk = struct.pack('>I', 9) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)
    ihdr_end = 8 + 4 + 4 + 13 + 4          # signature + IHDR length, type, data, CRC
    return png[:ihdr_end] + chunk + png[ihdr_end:]


def read_dpi(png: bytes) -> float | None:
    """pHYs density in dots per inch, or None (used by tests)."""
    pos = 8
    while pos < len(png):
        length, kind = struct.unpack('>I4s', png[pos:pos + 8])
        if kind == b'pHYs':
            x, _, unit = struct.unpack('>IIB', png[pos + 8:pos + 17])
            return x * INCH if unit == 1 else None
        pos += 12 + length
    return None


if __name__ == '__main__':
    sys.exit(main())
