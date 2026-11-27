"""Publish a calibration result (01-architecture §9).

  ros2 launch handeye_autopilot result.launch.py result_file:=<run_dir>/result.yaml

Runs ``publish_result``, which refuses a result whose ownership checks failed
when the run finished, re-checks the live TF tree (the camera root must still
have no parent, and no cycle), and only then publishes ``parent -> camera_root``
(the camera subtree rule) as a static transform. A result that failed its
quality gates is refused unless ``allow_failed:=true``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    allow = PythonExpression(["'--allow-failed' if '", LaunchConfiguration('allow_failed'),
                              "'.lower() in ('1', 'true', 'yes') else ''"])
    pub = Node(package='handeye_autopilot', executable='publish_result',
               name='handeye_result_publisher', output='screen',
               arguments=[LaunchConfiguration('result_file'), allow])
    return LaunchDescription([
        DeclareLaunchArgument('result_file', description='result.yaml of a run'),
        DeclareLaunchArgument('allow_failed', default_value='false',
                              description='also publish a result that failed its quality '
                                          'gates (not recommended)'),
        pub,
        RegisterEventHandler(OnProcessExit(target_action=pub,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
