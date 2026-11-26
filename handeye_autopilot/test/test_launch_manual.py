"""M3 done-when: launch_testing completes a manual-mode calibration with the
synthetic camera and scripted captures, using the installed executables and
``calibrate.launch.py`` exactly as a user would.

ROS 2 concept: ``launch_testing`` starts a launch description, then runs test
code against the live processes
(https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html).
"""

import math
import os

import pytest

pytest.importorskip('launch_testing')        # ROS-free CI jobs collect this file too

import tempfile
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.boards import BoardSpec
from handeye_autopilot.core.camera import CameraModel
from handeye_autopilot.ros.synthetic_camera import OPTICAL_RPY, pose
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy.time
from ros_harness import Driver, FakeRobot, camera_params, scripted_poses
from scipy.spatial.transform import Rotation
import yaml

from handeye_autopilot_interfaces.action import Calibrate

MODE = 'eye_to_hand'
CAM = camera_params(MODE)
OUT = tempfile.mkdtemp(prefix='handeye_launch_')


def _params_file():
    path = os.path.join(OUT, 'params.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump({'handeye_autopilot': {'ros__parameters': {
            'output_dir': OUT,
            'session': {'target_captures': 10, 'frames_per_capture': 2},
            'capture': {'consensus_min_frames': 2},
            'solver': {'min_captures': 8},
            'timing': {'settle_window': 0.3, 'settle_timeout': 3.0}}}}, f)
    return path


@pytest.mark.launch_test
def generate_test_description():
    share = get_package_share_directory('handeye_autopilot')
    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'calibrate.launch.py')),
        launch_arguments={'params_file': _params_file(), 'image': '/camera/image_raw',
                          'camera_info': '/camera/camera_info'}.items())
    camera = launch_ros.actions.Node(
        package='handeye_autopilot', executable='synthetic_camera', name='synthetic_camera',
        parameters=[CAM], remappings=[('image', '/camera/image_raw'),
                                      ('camera_info', '/camera/camera_info')])
    return launch.LaunchDescription([calibrate, camera,
                                     launch_testing.actions.ReadyToTest()])


class TestManualCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.robot = FakeRobot()
        cls.client = Node('launch_test_client')
        cls.driver = Driver(cls.client)
        cls.ex = MultiThreadedExecutor(num_threads=4)
        cls.ex.add_node(cls.robot)
        cls.ex.add_node(cls.client)
        threading.Thread(target=cls.ex.spin, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.ex.shutdown(timeout_sec=2.0)
        rclpy.shutdown()

    def test_manual_calibration_end_to_end(self):
        X = pose(CAM['true_camera_xyz_rpy']) @ tf.make(
            Rotation.from_euler('xyz', OPTICAL_RPY), np.zeros(3))
        Z = pose(CAM['true_board_xyz_rpy'])
        camera = CameraModel(np.array([[600.0, 0, 319.5], [0, 600.0, 239.5], [0, 0, 1]]),
                             np.zeros(5), 640, 480)
        board = BoardSpec('charuco', 7, 5, 0.04, 0.03, 'DICT_5X5_100')
        poses = scripted_poses(MODE, X, Z, camera, board, n=14)
        self.robot.move_to(poses[0])
        time.sleep(2.0)
        d = self.driver
        h = d.send(mode=MODE, timeout=30.0)
        self.assertTrue(h.accepted)
        d.wait_phase(Calibrate.Feedback.PHASE_MANUAL_CAPTURE, timeout=30.0)
        for G in poses:
            if d.feedback[-1].accepted >= 10:
                break
            self.robot.move_to(G)
            d.capture_and_wait()
        res = d.wait(h.get_result_async(), 120.0).result
        self.assertEqual(res.error_code, Calibrate.Result.NONE, res.message)
        t = res.camera_transform.transform.translation
        err = np.linalg.norm(np.array([t.x, t.y, t.z]) - X[:3, 3])
        self.assertLess(err, 0.003)
        q = res.camera_transform.transform.rotation
        Rm = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        ang = Rotation.from_matrix(Rm.T @ X[:3, :3]).magnitude()
        self.assertLess(math.degrees(ang), 0.5)
        self.assertTrue(res.passed, res.message)                    # quality gates
        zt = res.target_transform.transform.translation
        self.assertLess(np.linalg.norm(np.array([zt.x, zt.y, zt.z]) - Z[:3, 3]), 0.003)
        self.assertEqual(res.target_transform.header.frame_id, 'tool0')
        doc = yaml.safe_load(open(res.result_file))
        self.assertTrue(doc['publish']['ok'], doc['publish'].get('reason'))

        # the hand-off: result.launch.py publishes base_link -> camera_link (the
        # driver's subtree root) and TF then gives the true camera mount
        import subprocess

        import tf2_ros
        proc = subprocess.Popen(['ros2', 'launch', 'handeye_autopilot', 'result.launch.py',
                                 f'result_file:={res.result_file}'],
                                start_new_session=True)      # own process group
        try:
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, self.client)
            t0 = time.monotonic()
            got = None
            while time.monotonic() - t0 < 20 and got is None:
                try:
                    got = buf.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
                except tf2_ros.TransformException:
                    time.sleep(0.2)
            self.assertIsNotNone(got, 'result.launch.py published nothing')
            p = got.transform.translation
            true_root = pose(CAM['true_camera_xyz_rpy'])
            self.assertLess(np.linalg.norm(np.array([p.x, p.y, p.z]) - true_root[:3, 3]),
                            0.003)
            q = got.transform.rotation
            Rr = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            self.assertLess(math.degrees(Rotation.from_matrix(Rr.T @ true_root[:3, :3])
                                         .magnitude()), 0.5)
        finally:
            # stop the launch AND its children (a surviving publisher would keep
            # broadcasting this static transform into later tests)
            import signal
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
