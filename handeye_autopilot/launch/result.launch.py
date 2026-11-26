"""Publish a calibration result (01-architecture §9).

  ros2 launch handeye_autopilot result.launch.py result_file:=<run_dir>/result.yaml

Publishes ``parent -> camera_root`` (the camera subtree rule) with tf2's
``static_transform_publisher``. A result whose ownership checks failed (its
``publish.ok`` is false) is refused with the reason, so nothing conflicting is
ever published.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _publish(context):
    path = os.path.expanduser(LaunchConfiguration('result_file').perform(context))
    try:
        with open(path) as f:
            doc = yaml.safe_load(f)
    except OSError as e:
        return [LogInfo(msg=f'cannot read {path}: {e}'), Shutdown(reason='no result file')]
    pub = (doc or {}).get('publish') or {}
    if not pub.get('ok'):
        reason = pub.get('reason', 'no publish section')
        return [LogInfo(msg=f'refusing to publish {path}: {reason}'),
                Shutdown(reason='result not publishable')]
    t = pub['transform']
    xyz, q = t['translation'], t['rotation_xyzw']
    args = ['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
            '--frame-id', t['parent'], '--child-frame-id', t['child']]
    return [LogInfo(msg=f'publishing {t["parent"]} -> {t["child"]} from {path}'),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='handeye_result_publisher', arguments=args, output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('result_file', description='result.yaml of a run'),
        OpaqueFunction(function=_publish),
    ])
