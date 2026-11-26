"""Publish a calibration result as static TF, re-checking ownership NOW
(01-architecture §9).

    ros2 run handeye_autopilot publish_result <run_dir>/result.yaml

The run's ownership checks were made when it finished; the TF tree may have
changed since (a URDF publisher started, another calibration published). So
this listens to TF briefly, refuses if the camera root already has a parent
or would close a cycle, and only then publishes ``parent -> root`` with
tf2's StaticTransformBroadcaster (latched on /tf_static).
"""

from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
import tf2_ros
import yaml

from ..core import transforms as tf
from . import tf_tools


def check(tree: dict[str, str], parent: str, root: str) -> str:
    """'' if ``parent -> root`` may be published, else the reason."""
    if root in tree:
        return (f'{root!r} already has a parent ({tree[root]!r}) in the live TF tree; '
                'remove that transform first')
    if root in tf_tools.ancestors(tree, parent):
        return f'{root!r} is an ancestor of {parent!r}: publishing would make a cycle'
    return ''


class ResultPublisher(Node):
    def __init__(self, path: str, listen: float = 2.0) -> None:
        super().__init__('handeye_result_publisher')
        doc = yaml.safe_load(open(path)) or {}
        pub = doc.get('publish') or {}
        if not pub.get('ok'):
            raise SystemExit(f'refusing to publish {path}: '
                             f'{pub.get("reason", "no publish section")}')
        t = pub['transform']
        self.parent, self.root = t['parent'], t['child']
        T = tf.make(Rotation.from_quat(t['rotation_xyzw']), np.asarray(t['translation']))
        self.msg = tf_tools.to_msg(T, self.parent, self.root)
        self.subtree = pub.get('subtree')          # root -> optical at calibration time
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.listen = listen
        self.static = tf2_ros.StaticTransformBroadcaster(self)

    def check_subtree(self) -> str:
        """The live root -> optical must equal the one the result was computed with
        (a camera driver restarted with other extrinsics would move the optical
        frame by that difference)."""
        st = self.subtree
        got = tf_tools.latest(self.buffer, st['parent'], st['child'])
        if got is None:
            return (f'the camera subtree {st["parent"]} -> {st["child"]} is not in TF now: '
                    'start the camera driver first')
        then = tf.make(Rotation.from_quat(st['rotation_xyzw']), np.asarray(st['translation']))
        if tf.translation_distance(got[0], then) > 0.001 or \
                tf.rotation_angle(got[0], then) > np.radians(0.1):
            return (f'the camera subtree {st["parent"]} -> {st["child"]} changed since the '
                    'calibration (driver extrinsics differ); recalibrate')
        return ''

    def run(self) -> int:
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.listen:          # let /tf_static arrive
            rclpy.spin_once(self, timeout_sec=0.1)
        reason = check(tf_tools.parents(self.buffer), self.parent, self.root)
        if not reason and self.subtree is not None:
            reason = self.check_subtree()
        if reason:
            self.get_logger().error(f'not publishing {self.parent} -> {self.root}: {reason}')
            return 1
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.static.sendTransform(self.msg)
        self.get_logger().info(f'publishing {self.parent} -> {self.root}')
        rclpy.spin(self)
        return 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    if len(argv) != 2:
        raise SystemExit('usage: publish_result <result.yaml>')
    code = 1
    try:
        node = ResultPublisher(argv[1])
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 0
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
