"""Camera input (01-architecture §7): CameraInfo validation and a recent-image buffer.

Reuse: ``cv_bridge`` for image conversion; the camera model itself is the core
``CameraModel`` (K, D as NumPy, OpenCV projection), built here from
``CameraInfo`` with the §7 rules. No ROS types leave this module except the
messages it is given.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
import threading

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import numpy as np
from numpy.typing import NDArray
from sensor_msgs.msg import CameraInfo, Image

from ..core.camera import CameraModel, CameraModelError


class CameraInfoError(ValueError):
    pass


def camera_model(info: CameraInfo, rectified: bool) -> CameraModel:
    """§7: raw -> K, D (plumb_bob / rational_polynomial); rectified -> P[:3,:3],
    no distortion, R must be identity. Uncalibrated, binned or ROI'd refused.
    The message is parsed with image_geometry.PinholeCameraModel (reuse map)."""
    pin = PinholeCameraModel()
    pin.from_camera_info(info)
    K = np.asarray(pin.intrinsic_matrix(), dtype=float).reshape(3, 3)
    if not np.any(K):
        raise CameraInfoError('CameraInfo K is all zero: the camera is not calibrated '
                              '(run camera_calibration first)')
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1):
        # shortcut: binning unsupported in v1; scale K when someone needs it
        raise CameraInfoError('binned CameraInfo is not supported in v1')
    roi = info.roi
    full = (roi.x_offset == 0 and roi.y_offset == 0 and
            roi.width in (0, info.width) and roi.height in (0, info.height))
    if not full:
        # shortcut: ROI unsupported in v1; shift the principal point when needed
        raise CameraInfoError('a CameraInfo ROI is not supported in v1')
    try:
        if rectified:
            R = np.asarray(pin.rotation_matrix(), dtype=float).reshape(3, 3)
            if not np.allclose(R, np.eye(3), atol=1e-9):
                # shortcut: stereo-rectified R unsupported; rotate the result by R
                raise CameraInfoError('image_is_rectified needs R = identity in CameraInfo '
                                      '(monocular rectification); stereo rectification is '
                                      'not supported in v1')
            P = np.asarray(pin.projection_matrix(), dtype=float).reshape(3, 4)
            return CameraModel(P[:, :3].copy(), np.zeros(0), int(info.width),
                               int(info.height), 'plumb_bob', rectified=True)
        raw_d = pin.distortion_coeffs()
        D = np.zeros(0) if raw_d is None else np.asarray(raw_d, dtype=float).ravel()
        return CameraModel(K, D, int(info.width), int(info.height),
                           info.distortion_model or 'plumb_bob')
    except CameraModelError as e:
        raise CameraInfoError(str(e)) from e


def same_info(a: CameraInfo, b: CameraInfo) -> bool:
    """Everything the camera model depends on is unchanged (a change mid-run
    aborts it, §7). Never raises (arrays of different lengths are just unequal)."""
    def eq(x: object, y: object) -> bool:
        xa, ya = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
        return xa.shape == ya.shape and bool(np.allclose(xa, ya))
    return (a.header.frame_id == b.header.frame_id and a.width == b.width and
            a.height == b.height and a.distortion_model == b.distortion_model and
            a.binning_x == b.binning_x and a.binning_y == b.binning_y and
            (a.roi.x_offset, a.roi.y_offset, a.roi.width, a.roi.height) ==
            (b.roi.x_offset, b.roi.y_offset, b.roi.width, b.roi.height) and
            eq(a.k, b.k) and eq(a.d, b.d) and eq(a.p, b.p) and eq(a.r, b.r))


@dataclass
class StampedImage:
    stamp: float                      # acquisition stamp, s (header.stamp)
    stamp_ns: int                     # the same, exact (for TF lookups)
    frame_id: str
    gray: NDArray[np.uint8]
    received: float                   # node clock when it arrived (clock-domain check)


class ImageBuffer:
    """The last few images, converted to grayscale, newest last. Thread-safe."""

    def __init__(self, clock: Callable[[], float], size: int = 90) -> None:
        self._clock = clock
        self._buf: deque[StampedImage] = deque(maxlen=size)
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._bridge = CvBridge()

    def add(self, msg: Image, msg_info: dict[str, int] | None = None) -> None:
        """``msg_info`` (rclpy MessageInfo): its ``received_timestamp`` is when the
        middleware got the message (system clock, ns); the callback may run later
        under load, so that lag is subtracted from the node-clock arrival time."""
        import time
        lag = 0.0
        if msg_info and msg_info.get('received_timestamp'):
            lag = max(0.0, (time.time_ns() - int(msg_info['received_timestamp'])) * 1e-9)
        gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._cond:
            self._buf.append(StampedImage(ns * 1e-9, ns, msg.header.frame_id,
                                          np.asarray(gray), self._clock() - lag))
            self._cond.notify_all()

    def latest(self) -> StampedImage | None:
        with self._lock:
            return self._buf[-1] if self._buf else None

    def wait_received_after(self, after: float, count: int, timeout: float,
                            cancelled: threading.Event | None = None) -> list[StampedImage]:
        """Up to ``count`` images that ARRIVED after node time ``after`` (oldest
        first), waiting up to ``timeout`` s of wall time. Selecting by arrival
        (not by stamp) means a camera on another clock is still seen, so its
        stamps can be checked and refused (§8) instead of silently filtered."""
        import time
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                got = [im for im in self._buf if im.received > after]
                if len(got) >= count:
                    return got[:count]
                left = deadline - time.monotonic()
                if left <= 0 or (cancelled is not None and cancelled.is_set()):
                    return got
                self._cond.wait(min(left, 0.05))

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()
