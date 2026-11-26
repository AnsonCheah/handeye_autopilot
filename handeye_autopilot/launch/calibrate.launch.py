"""Start the calibration node (01-architecture §5a).

  ros2 launch handeye_autopilot calibrate.launch.py \
      image:=/camera/image_raw camera_info:=/camera/camera_info [record:=true]

ROS 2 concept: a launch file starts and configures several processes at once
(https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html).
"""

from datetime import UTC, datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _record(context):
    if LaunchConfiguration('record').perform(context).lower() not in ('1', 'true', 'yes'):
        return []
    out = os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))
    name = datetime.now(UTC).strftime('bag_%Y%m%dT%H%M%SZ')
    topics = [LaunchConfiguration('image').perform(context),
              LaunchConfiguration('camera_info').perform(context),
              '/tf', '/tf_static', '/joint_states', '/handeye_autopilot/annotated_image']
    return [ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', os.path.join(out, name)]
                           + topics, output='screen')]


def generate_launch_description():
    share = get_package_share_directory('handeye_autopilot')
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'default.yaml')),
        DeclareLaunchArgument('image', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info', default_value='/camera/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('record', default_value='false',
                              description='also record a rosbag of the run'),
        DeclareLaunchArgument('bag_dir', default_value='~/.ros/handeye_autopilot'),
        Node(package='handeye_autopilot', executable='calibrate_node',
             name='handeye_autopilot', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[('image', LaunchConfiguration('image')),
                         ('camera_info', LaunchConfiguration('camera_info'))]),
        OpaqueFunction(function=_record),
    ])
