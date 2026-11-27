"""Visualisation (02-features §I): annotated image and RViz markers.

Reuse: OpenCV drawing, ``cv_bridge``, ``visualization_msgs/MarkerArray``. We
write only the two builders below.
"""

from __future__ import annotations

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from ..core import transforms as tf
from ..core.boards import BoardSpec
from ..core.capture import CaptureRecord

_bridge = CvBridge()


def annotated_image(gray: NDArray[np.uint8], rec: CaptureRecord, board: BoardSpec,
                    header: object, text: str = '') -> Image:
    """Detected corners (green if accepted, red if not), the board outline from the
    refined pose, and the outcome text."""
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    color = (0, 200, 0) if rec.accepted else (0, 0, 230)
    if rec.image_points is not None:
        for u, v in np.asarray(rec.image_points):
            cv2.circle(img, (int(round(u)), int(round(v))), 3, color, -1)
    if rec.camera_T_board is not None:
        px = rec.camera.project_object(rec.camera_T_board, board.outline_samples(20))
        cv2.polylines(img, [np.round(px).astype(np.int32)], True, color, 2)
    label = text or ('accepted' if rec.accepted else f'rejected: {rec.reject}')
    cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    if not rec.accepted and rec.detail:
        cv2.putText(img, rec.detail[:90], (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    msg = _bridge.cv2_to_imgmsg(img, encoding='bgr8')
    msg.header = header  # type: ignore[assignment]
    return msg


def board_markers(rec: CaptureRecord, board: BoardSpec, frame_id: str, stamp: object,
                  ns: str = 'board') -> MarkerArray:
    """The board outline at the capture's refined pose, in the optical frame."""
    arr = MarkerArray()
    if rec.camera_T_board is None:
        return arr
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp  # type: ignore[assignment]
    m.ns, m.id, m.type, m.action = ns, 0, Marker.LINE_STRIP, Marker.ADD
    m.scale.x = 0.003
    m.color.a = 1.0
    m.color.g = 0.8 if rec.accepted else 0.0
    m.color.r = 0.0 if rec.accepted else 0.9
    m.pose.orientation.w = 1.0
    pts = tf.apply(rec.camera_T_board, board.outline)
    for p in list(pts) + [pts[0]]:
        m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
    arr.markers.append(m)
    return arr


def box_marker(centre: NDArray[np.float64], size: tuple[float, float, float], frame_id: str,
               stamp: object) -> MarkerArray:
    """The safety box (01-architecture §3.4), translucent. ``shortcut: not editable
    in RViz, add an interactive marker if users ask``."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp  # type: ignore[assignment]
    m.ns, m.id, m.type, m.action = 'safety_box', 0, Marker.CUBE, Marker.ADD
    q = Rotation.from_matrix(centre[:3, :3]).as_quat()
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in centre[:3, 3])
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
        float(v) for v in q)
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in size)
    m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.6, 1.0, 0.12
    return MarkerArray(markers=[m])
