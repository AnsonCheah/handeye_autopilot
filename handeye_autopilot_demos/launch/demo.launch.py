"""handeye_autopilot demo: a robot on mock hardware with MoveIt, a synthetic
camera, the calibration node, and the demo scene.

    ros2 launch handeye_autopilot_demos demo.launch.py robot:=panda mode:=eye_to_hand
    ros2 launch handeye_autopilot_demos demo.launch.py robot:=ur mode:=eye_in_hand rviz:=true

Arguments:
- ``robot``: ``panda`` or ``ur`` (a UR5e);
- ``mode``: ``eye_to_hand`` or ``eye_in_hand``;
- ``auto`` (default true): ``run_demo`` moves to the seed, calibrates unattended,
  prints DEMO PASSED or DEMO FAILED, and the whole launch then stops (its own
  exit code is 0 either way: read the verdict, or run ``run_demo`` yourself,
  which exits 0 or 1); false leaves everything running for you to send goals
  (README "try it");
- ``rviz`` (default false): RViz with the robot, the planning scene, the
  safety box and the annotated camera image;
- ``record`` (default false): ``ros2 bag record`` of the run (calibrate.launch.py).

ROS 2 concept: a launch file starts and configures several nodes at once
(https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html);
``OpaqueFunction`` reads the argument values to pick the robot's config.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('eye_to_hand', 'eye_in_hand'):
        raise RuntimeError(f'mode must be eye_to_hand or eye_in_hand, got {mode!r}')
    demos = get_package_share_directory('handeye_autopilot_demos')
    calib = get_package_share_directory('handeye_autopilot')
    with open(os.path.join(demos, 'config', f'{robot}.yaml')) as f:
        cfg = yaml.safe_load(f)
    r = cfg['robot']
    truth = cfg[mode]
    frames = {'base_frame': r['base_frame'], 'tool_frame': r['tool_frame']}
    camera = Node(
        package='handeye_autopilot', executable='synthetic_camera', name='synthetic_camera',
        output='log',
        parameters=[dict(frames, mode=mode, **truth,
                         **{f'board.{k}': v for k, v in cfg['board'].items()})],
        remappings=[('image', '/camera/image_raw'), ('camera_info', '/camera/camera_info')])
    # the calibration node's parameters: the demo's frames, board, MoveIt group
    params = dict(cfg.get('calibration', {}))
    params.update(frames)
    params['board'] = dict(cfg['board'])
    params['moveit'] = dict(params.get('moveit', {}), group_name=r['group_name'],
                            ik_link=r['ik_link'], joint_names=list(r['joint_names']))
    params['output_dir'] = os.path.expanduser(f'~/.ros/handeye_autopilot/demo_{robot}_{mode}')
    fd, params_file = tempfile.mkstemp(prefix='handeye_demo_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump({'handeye_autopilot': {'ros__parameters': params}}, f)
    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(calib, 'launch', 'calibrate.launch.py')),
        launch_arguments={'params_file': params_file, 'image': '/camera/image_raw',
                          'camera_info': '/camera/camera_info',
                          'record': LaunchConfiguration('record')}.items())
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(demos, 'launch', r['launch'])))
    scene = Node(package='handeye_autopilot_demos', executable='scene', output='log',
                 parameters=[{'robot': robot, 'mode': mode}])
    runner = Node(package='handeye_autopilot_demos', executable='run_demo', output='screen',
                  parameters=[{'robot': robot, 'mode': mode}],
                  condition=IfCondition(LaunchConfiguration('auto')))
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(demos, 'rviz', 'demo.rviz'),
                           '-f', r['base_frame']],
                condition=IfCondition(LaunchConfiguration('rviz')))
    stop_with_runner = RegisterEventHandler(OnProcessExit(
        target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason='demo finished'))]))
    return [robot_launch, camera, calibrate, scene, runner, rviz, stop_with_runner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='panda', choices=['panda', 'ur']),
        DeclareLaunchArgument('mode', default_value='eye_to_hand',
                              choices=['eye_to_hand', 'eye_in_hand']),
        DeclareLaunchArgument('auto', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('record', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
