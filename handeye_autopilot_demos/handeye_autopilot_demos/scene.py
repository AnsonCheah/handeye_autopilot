"""Demo planning scene: a table, the carried board or camera as an attached
collision object, the fixed one as a world object, and a point-cloud obstacle.

    ros2 run handeye_autopilot_demos scene --ros-args -p robot:=panda -p mode:=eye_to_hand

MoveIt concepts: the *planning scene* is MoveIt's model of the world that
planning and ``compute_ik`` check collisions against; an *attached* collision
object moves with a robot link, so poses where only the carried board collides
are rejected too
(https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html).
The point cloud goes through MoveIt's octomap updater (``config/sensors_3d.yaml``),
like a depth camera would.

Reuse: ``moveit_msgs`` ApplyPlanningScene and ``sensor_msgs_py.point_cloud2``.
We write the demo's object list.
"""

from __future__ import annotations

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.boards import BoardSpec
from handeye_autopilot.ros.moveit_client import to_pose
from handeye_autopilot.ros.synthetic_camera import pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Header
import yaml

CAMERA_BODY = (0.09, 0.03, 0.03)       # m, a small camera's housing (x along its view)


def load(robot: str) -> dict[str, Any]:
    path = os.path.join(get_package_share_directory('handeye_autopilot_demos'), 'config',
                        f'{robot}.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


def box(name: str, frame: str, T: np.ndarray, size: list[float]) -> CollisionObject:
    o = CollisionObject()
    o.header.frame_id, o.id = frame, name
    o.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])]
    o.primitive_poses = [to_pose(T)]
    o.operation = CollisionObject.ADD
    return o


def board_box(spec: dict[str, Any], frame: str, T_frame_board: np.ndarray) -> CollisionObject:
    b = BoardSpec(spec['kind'], spec['squares_x'], spec['squares_y'], spec['square_length'],
                  spec.get('marker_length', 0.0), spec.get('dictionary', ''))
    w, h = b.size_m
    centre = T_frame_board @ tf.make(np.eye(3), np.array([w / 2, h / 2, 0.004]))
    return box('calibration_board', frame, centre, [w + 0.01, h + 0.01, 0.008])


def camera_box(frame: str, T_frame_root: np.ndarray) -> CollisionObject:
    # camera_link: x along the view; the housing sits behind the lens
    return box('camera', frame, T_frame_root @ tf.make(np.eye(3), np.array([-0.04, 0, 0])),
               list(CAMERA_BODY))


def pillar_cloud(xyz: list[float], size: list[float], step: float = 0.02) -> np.ndarray:
    """Points on the faces of a box: what a depth camera would see of a pillar."""
    c, s = np.asarray(xyz, float), np.asarray(size, float) / 2
    axes = [np.arange(-v, v + 1e-9, step) for v in s]
    pts = []
    for i in range(3):
        for sign in (-1.0, 1.0):
            j, k = [a for a in range(3) if a != i]
            gj, gk = np.meshgrid(axes[j], axes[k])
            p = np.zeros((gj.size, 3))
            p[:, i], p[:, j], p[:, k] = sign * s[i], gj.ravel(), gk.ravel()
            pts.append(p)
    return np.vstack(pts) + c


class Scene(Node):
    def __init__(self) -> None:
        super().__init__('demo_scene')
        robot = self.declare_parameter('robot', 'panda').value
        self.mode = self.declare_parameter('mode', 'eye_to_hand').value
        cfg = load(robot)
        r = cfg['robot']
        self.base = r['base_frame']
        truth = cfg[self.mode]
        cam, brd = pose(truth['true_camera_xyz_rpy']), pose(truth['true_board_xyz_rpy'])
        table = pose(list(r['table']['xyz']) + [0.0, 0.0, 0.0])
        world: list[CollisionObject] = [box('table', self.base, table, r['table']['size'])]
        for extra in r.get('boxes', []):        # e.g. a pedestal under the robot
            world.append(box(extra['name'], self.base,
                             pose(list(extra['xyz']) + [0.0, 0.0, 0.0]), extra['size']))
        attached = AttachedCollisionObject(link_name=r['tool_frame'],
                                           touch_links=list(r['touch_links']))
        if self.mode == 'eye_to_hand':          # camera fixed in the world, board on the tool
            world.append(camera_box(self.base, cam))
            attached.object = board_box(cfg['board'], r['tool_frame'], brd)
        else:                                   # board fixed in the world, camera on the tool
            world.append(board_box(cfg['board'], self.base, brd))
            attached.object = camera_box(r['tool_frame'], cam)
        self.scene = PlanningScene(is_diff=True)
        self.scene.world.collision_objects = world
        self.scene.robot_state.is_diff = True
        self.scene.robot_state.attached_collision_objects = [attached]
        ob = r['obstacle']
        self.cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=self.base),
                                                     pillar_cloud(ob['xyz'], ob['size']).tolist())
        self.pub_cloud = self.create_publisher(PointCloud2, '/demo/obstacle_points', 2)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ready = self.create_publisher(Bool, '/demo/scene_ready', latched)
        self.apply = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.create_timer(0.5, self._publish_cloud)
        self._applied = False
        self._future: Any = None
        self.create_timer(1.0, self._try_apply)

    def _publish_cloud(self) -> None:
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.pub_cloud.publish(self.cloud)

    def _try_apply(self) -> None:
        if self._applied:
            return
        if self._future is not None and self._future.done():
            ok = self._future.result() is not None and self._future.result().success
            self._future = None
            if ok:
                self._applied = True
                self.get_logger().info(f'planning scene ready ({self.mode})')
                self.pub_ready.publish(Bool(data=True))
                return
        if self._future is None and self.apply.service_is_ready():
            self._future = self.apply.call_async(ApplyPlanningScene.Request(scene=self.scene))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Scene()
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
