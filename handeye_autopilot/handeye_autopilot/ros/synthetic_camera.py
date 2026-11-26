"""Synthetic camera node (01-architecture §5a, §10): a fake camera driver.

    ros2 run handeye_autopilot synthetic_camera --ros-args -p mode:=eye_to_hand ...

Publishes ``image`` + ``camera_info`` at ``rate`` Hz, rendering the board with
the core homography renderer at the pose the TRUE mounts give (parameters,
never published as TF: they are what the calibration must find) and the latest
``base -> tool`` TF. Like a real driver it publishes its own rigid subtree
``camera_root_frame -> optical_frame`` on /tf_static, so the subtree rule (§9)
is exercised. Each image is stamped with the TF stamp of the robot pose it
was rendered at, so image time and robot pose correspond exactly.
``stamp_from_clock`` stamps with the node clock instead (a camera whose time
does not depend on TF, for delayed-TF tests); ``stamp_offset`` (s) shifts
image stamps to test the clock preflight. Limits of the renderer: no lens
shading, no occlusion, no motion blur (02-features §H).
"""

from __future__ import annotations

import math
from typing import Any

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from ..core import handeye
from ..core import transforms as tf
from ..core.boards import BoardSpec, board_texture
from ..core.camera import CameraModel
from ..core.render import ImageEffects, PlanarObject, Renderer
from . import tf_tools

# optical frame convention (REP 103): z forward, x right, y down, from a
# camera_link with x forward, y left, z up
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)


def pose(xyz_rpy: list[float]) -> np.ndarray:
    return tf.make(Rotation.from_euler('xyz', xyz_rpy[3:]), np.asarray(xyz_rpy[:3], float))


class SyntheticCamera(Node):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('synthetic_camera', **kwargs)
        d = self.declare_parameter
        d('mode', 'eye_to_hand')
        d('base_frame', 'base_link')
        d('tool_frame', 'tool0')
        d('camera_root_frame', 'camera_link')
        d('optical_frame', 'camera_optical_frame')
        d('rate', 10.0)
        d('width', 640)
        d('height', 480)
        d('fx', 600.0)
        d('fy', 600.0)
        d('cx', 319.5)
        d('cy', 239.5)
        d('distortion', [0.0, 0.0, 0.0, 0.0, 0.0])
        # truth: parent -> camera_root (parent = base for eye_to_hand, tool for
        # eye_in_hand) and board mount (tool for eye_to_hand, base for eye_in_hand)
        d('true_camera_xyz_rpy', [0.6, 0.0, 0.5, 0.0, 0.6, math.pi])
        d('true_board_xyz_rpy', [0.0, 0.0, 0.05, 0.0, 0.0, 0.0])
        d('board.kind', 'charuco')
        d('board.squares_x', 7)
        d('board.squares_y', 5)
        d('board.square_length', 0.04)
        d('board.marker_length', 0.03)
        d('board.dictionary', 'DICT_5X5_100')
        d('noise_sigma', 2.0)
        d('stamp_offset', 0.0)
        d('stamp_from_clock', False)   # stamp with the node clock, not the pose's TF stamp
        d('seed', 0)

        g = self.get_parameter
        self.mode = g('mode').value
        if self.mode not in ('eye_to_hand', 'eye_in_hand'):
            raise ValueError(f'mode must be eye_to_hand or eye_in_hand, got {self.mode!r}')
        self.base, self.tool = g('base_frame').value, g('tool_frame').value
        self.root, self.optical = g('camera_root_frame').value, g('optical_frame').value
        K = np.array([[g('fx').value, 0, g('cx').value], [0, g('fy').value, g('cy').value],
                      [0, 0, 1.0]])
        self.camera = CameraModel(K, np.asarray(g('distortion').value, float),
                                  int(g('width').value), int(g('height').value))
        self.board = BoardSpec(g('board.kind').value, int(g('board.squares_x').value),
                               int(g('board.squares_y').value),
                               float(g('board.square_length').value),
                               float(g('board.marker_length').value),
                               g('board.dictionary').value)
        self.T_parent_root = pose(list(g('true_camera_xyz_rpy').value))
        self.T_root_optical = tf.make(Rotation.from_euler('xyz', OPTICAL_RPY), np.zeros(3))
        self.Z = pose(list(g('true_board_xyz_rpy').value))
        self.offset = float(g('stamp_offset').value)
        self.stamp_from_clock = bool(g('stamp_from_clock').value)
        self.rng = np.random.default_rng(int(g('seed').value))
        tex, H = board_texture(self.board, 60, 30)
        self.texture, self.tex_H = tex, H
        self.renderer = Renderer(self.camera, ImageEffects(
            noise_sigma=float(g('noise_sigma').value)))
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static = tf2_ros.StaticTransformBroadcaster(self)
        self.static.sendTransform(tf_tools.to_msg(self.T_root_optical, self.root, self.optical,
                                                  self.get_clock().now().to_msg()))
        self.pub_img = self.create_publisher(Image, 'image', qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, 'camera_info',
                                              qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / float(g('rate').value), self._tick)

    @property
    def true_X(self) -> np.ndarray:
        """parent_T_optical: what the calibration should recover (camera_transform)."""
        return self.T_parent_root @ self.T_root_optical

    def board_in_camera(self, G: np.ndarray) -> np.ndarray:
        """camera_T_board from the true mounts and base_T_tool = G."""
        return handeye.predict_board(self.mode, self.true_X, self.Z, G)

    def info(self, stamp: Any) -> CameraInfo:
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, self.optical
        m.width, m.height = self.camera.width, self.camera.height
        m.distortion_model = 'plumb_bob'
        m.d = [float(v) for v in self.camera.D]
        m.k = [float(v) for v in self.camera.K.ravel()]
        m.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        m.p = [float(v) for v in np.c_[self.camera.K, np.zeros(3)].ravel()]
        return m

    def _tick(self) -> None:
        got = tf_tools.latest(self.tf_buffer, self.base, self.tool)
        if got is None:
            return                                   # no robot yet: no image
        G, tf_stamp = got
        C = self.board_in_camera(G)
        img = self.renderer.render([PlanarObject(self.texture, self.tex_H, C)], self.rng)
        # the image shows the robot at the pose's TF stamp: stamp it with that
        # instant (an exposure at time t shows the robot at t), plus any offset
        t = self.get_clock().now().nanoseconds * 1e-9 if self.stamp_from_clock else tf_stamp
        stamp = tf_tools.stamp_to_time(t + self.offset).to_msg()
        msg = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        msg.header.stamp, msg.header.frame_id = stamp, self.optical
        self.pub_img.publish(msg)
        self.pub_info.publish(self.info(stamp))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SyntheticCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
