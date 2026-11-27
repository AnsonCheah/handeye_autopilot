"""Panda on mock hardware with MoveIt, headless (no RViz), for the demos.

Built from moveit_resources_panda_moveit_config's own files with
``MoveItConfigsBuilder`` (the package's demo.launch.py always starts RViz). Adds
the octomap sensor for the demo's point-cloud obstacle.

ROS 2 / MoveIt concepts: ``ros2_control`` with ``mock_components`` mirrors
commanded joint positions back as measured ones (a robot without hardware,
https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html);
``move_group`` serves planning, ``compute_ik`` and the planning scene
(https://moveit.picknik.ai/main/doc/concepts/move_group.html).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    demos = get_package_share_directory('handeye_autopilot_demos')
    panda = get_package_share_directory('moveit_resources_panda_moveit_config')
    moveit_config = (
        MoveItConfigsBuilder('moveit_resources_panda')
        .robot_description(file_path='config/panda.urdf.xacro',
                           mappings={'ros2_control_hardware_type': 'mock_components'})
        .robot_description_semantic(file_path='config/panda.srdf')
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .trajectory_execution(file_path='config/moveit_controllers.yaml')   # arm only
        .planning_pipelines(pipelines=['ompl'])
        .sensors_3d(file_path=os.path.join(demos, 'config', 'sensors_3d.yaml'))
        .to_moveit_configs()
    )
    octomap = {'octomap_frame': 'panda_link0', 'octomap_resolution': 0.02,
               'max_range': 3.0}
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'world', '--child-frame-id', 'panda_link0'],
             output='log'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[moveit_config.robot_description], output='log'),
        Node(package='moveit_ros_move_group', executable='move_group', output='log',
             parameters=[moveit_config.to_dict(), octomap]),
        Node(package='controller_manager', executable='ros2_control_node', output='log',
             parameters=[os.path.join(panda, 'config', 'ros2_controllers.yaml')],
             remappings=[('/controller_manager/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['joint_state_broadcaster', '-c', '/controller_manager']),
        Node(package='controller_manager', executable='spawner', output='log',
             arguments=['panda_arm_controller', '-c', '/controller_manager']),
    ])
