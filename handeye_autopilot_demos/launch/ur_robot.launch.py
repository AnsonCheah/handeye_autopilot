"""UR5e on mock hardware with MoveIt, headless, for the demos.

The driver part is ``ur_robot_driver``'s own ``ur_control.launch.py`` with
``use_mock_hardware:=true`` (no robot or URSim needed). ``move_group`` is
started like ``ur_moveit_config``'s ``ur_moveit.launch.py`` does (after the
robot description is published), plus the octomap sensor for the demo's
point-cloud obstacle, which that launch file has no argument for.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

UR_TYPE = 'ur5e'


def generate_launch_description():
    demos = get_package_share_directory('handeye_autopilot_demos')
    driver = get_package_share_directory('ur_robot_driver')
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver, 'launch', 'ur_control.launch.py')),
        launch_arguments={'ur_type': UR_TYPE, 'robot_ip': '127.0.0.1',
                          'use_mock_hardware': 'true', 'launch_rviz': 'false',
                          'launch_dashboard_client': 'false',
                          'initial_joint_controller': 'scaled_joint_trajectory_controller',
                          }.items())
    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        .robot_description_semantic(os.path.join('srdf', 'ur.srdf.xacro'), {'name': UR_TYPE})
        .sensors_3d(file_path=os.path.join(demos, 'config', 'sensors_3d.yaml'))
        .to_moveit_configs()
    )
    octomap = {'octomap_frame': 'base_link', 'octomap_resolution': 0.02, 'max_range': 3.0}
    wait = Node(package='ur_robot_driver', executable='wait_for_robot_description',
                output='log')
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='log',
                      parameters=[moveit_config.to_dict(), octomap])
    return LaunchDescription([
        control, wait,
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=[move_group])),
    ])
