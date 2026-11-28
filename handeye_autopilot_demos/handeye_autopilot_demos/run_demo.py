"""Run one demo calibration unattended and report it.

    ros2 run handeye_autopilot_demos run_demo --ros-args -p robot:=panda -p mode:=eye_to_hand

What an operator does by hand, scripted: wait for MoveIt and the demo scene,
move the robot to the seed pose (on a real robot you jog there), send the
``Calibrate`` goal with ``execution: moveit``, print progress, and exit 0 only
if the run SUCCEEDED and passed every quality gate (1 otherwise). With
``check_truth`` it also compares the result with the synthetic camera's true
mount, which only a simulation knows.

ROS 2 concept: an action client sends a goal, receives feedback while it runs
and a result at the end (https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).
"""

from __future__ import annotations

import math
import sys
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.ros.moveit_client import MoveItClient, error_name, wait_future
from handeye_autopilot.ros.synthetic_camera import OPTICAL_RPY, pose
from moveit_msgs.srv import GetStateValidity
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from handeye_autopilot_demos.scene import load
from handeye_autopilot_interfaces.action import Calibrate


class Runner(Node):
    def __init__(self) -> None:
        super().__init__('demo_runner')
        self.robot = self.declare_parameter('robot', 'panda').value
        self.mode = self.declare_parameter('mode', 'eye_to_hand').value
        self.check_truth = self.declare_parameter('check_truth', True).value
        self.cfg = load(self.robot, self.declare_parameter('config', '').value)
        self.timeout = float(self.declare_parameter(
            'timeout', float(self.cfg.get('demo_timeout', 900.0))).value)
        cb = ReentrantCallbackGroup()
        self.moveit = MoveItClient(self, cb)
        self.action = ActionClient(self, Calibrate, '/handeye_autopilot/calibrate',
                                   callback_group=cb)
        # stops whatever run is active, whoever started it: the fallback when a
        # goal reply or a cancel reply is lost
        self.cancel_any = self.create_client(Trigger, '/handeye_autopilot/cancel',
                                             callback_group=cb)
        self.scene_ready = threading.Event()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/demo/scene_ready',
                                 lambda m: m.data and self.scene_ready.set(), latched,
                                 callback_group=cb)
        self.last_phase = ''

    def log(self, msg: str) -> None:
        self.get_logger().info(msg)

    def run(self) -> int:
        r = self.cfg['robot']
        self.log('waiting for MoveIt, the demo scene and the calibration node...')
        missing = self.moveit.missing(120.0)
        if missing:
            self.get_logger().error(f'MoveIt interfaces missing: {missing}')
            return 1
        if not self.scene_ready.wait(120.0) or not self.action.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('the demo scene or the calibration node did not start')
            return 1
        self.log('moving to the seed pose (an operator would jog the robot here)')
        goal = self.moveit.joint_goal(r['seed_joints'], r['joint_names'], r['group_name'],
                                      r['base_frame'], 10.0, 0.5, 0.5)
        # right after start-up move_group may not have the robot's state yet (its
        # plan then starts from a stale state and execution refuses it): retry
        for attempt in range(5):
            time.sleep(2.0)
            out = self.moveit.run(self.moveit.move, goal, 60.0, 5.0, threading.Event())
            if out.status == GoalStatus.STATUS_SUCCEEDED:
                break
            self.log(f'seed move attempt {attempt + 1} failed ({error_name(out.error_code)}), '
                     'retrying')
        if out.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'could not reach the seed: status {out.status}, '
                                    f'{error_name(out.error_code)} {out.note}')
            self._log_contacts(r)
            return 1
        time.sleep(2.0)                     # let TF and the camera catch up
        self.log(f'calibrating ({self.mode}, execution: moveit)')
        send = self.action.send_goal_async(
            Calibrate.Goal(mode=self.mode, execution='moveit'), feedback_callback=self._fb)
        if not wait_future(send, 30.0):
            self.get_logger().error('no reply to the calibration goal; cancelling any run')
            self._cancel_any()
            return 1
        if not send.result().accepted:
            self.get_logger().error('the calibration goal was rejected')
            return 1
        res_fut = send.result().get_result_async()
        if not wait_future(res_fut, self.timeout):
            self.get_logger().error('the calibration did not finish in time; cancelling it')
            wait_future(send.result().cancel_goal_async(), 10.0)
            if not wait_future(res_fut, 60.0):   # the node stops the robot first
                self._cancel_any(res_fut)
            else:
                self._log_end(res_fut.result())
            return 1
        res = res_fut.result()
        r_ = res.result
        q = r_.quality
        self.log(f'finished: status {res.status}, error_code {r_.error_code}: {r_.message}')
        self.log(f'captures accepted {q.captures_accepted}, rejected {q.captures_rejected}; '
                 f'reprojection RMS {q.reprojection_rms_px:.2f} px; held-out RMS '
                 f'{q.heldout_reprojection_rms_px:.2f} px; passed {r_.passed}')
        self.log(f'result file: {r_.result_file}')
        ok = res.status == GoalStatus.STATUS_SUCCEEDED and r_.error_code == 0 and r_.passed
        if ok and self.check_truth:
            ok = self._truth(r_)
        self.log('DEMO PASSED' if ok else 'DEMO FAILED')
        if ok:
            self.log('publish it to TF while the demo still runs (default with RViz):\n'
                     '  ros2 launch handeye_autopilot result.launch.py '
                     f'result_file:={r_.result_file}')
        return 0 if ok else 1

    def _cancel_any(self, res_fut: Any = None) -> None:
        """~/cancel: the node cancels its active run and stops the robot. It
        cancels whichever run is active: in the demo that is the runner's own
        (the demo's node serves nobody else). The reply comes before the robot
        has stopped, so wait for the run's result when there is one."""
        if not self.cancel_any.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('~/cancel is not available: stop the robot yourself')
            return
        fut = self.cancel_any.call_async(Trigger.Request())
        if not wait_future(fut, 60.0):
            self.get_logger().error('~/cancel did not reply: stop the robot yourself')
        else:
            self.get_logger().info(f'~/cancel: {fut.result().message}')
        if res_fut is not None:
            if not wait_future(res_fut, 120.0):
                self.get_logger().error('the run did not confirm the stop: check the robot')
            else:
                self._log_end(res_fut.result())

    def _log_end(self, res: Any) -> None:
        """How a cancelled run ended; an unknown goal status confirms nothing."""
        if res.status == GoalStatus.STATUS_UNKNOWN:
            self.get_logger().error('the node does not know the run (status unknown): '
                                    'the stop is not confirmed, check the robot')
            return
        self.get_logger().error(f'the run ended: status {res.status}, error_code '
                                f'{res.result.error_code}: {res.result.message}')

    def _log_contacts(self, r: dict) -> None:
        """Which bodies collide, at the current state and at the seed (MoveIt's
        check_state_validity): the sweep counts a seed failure as a mount problem
        only if the board or the camera is one of them."""
        # 'current state': the state MoveIt just refused to start from; nothing
        # was executed, so the robot is still there (the sweep reads these
        # contacts for START_STATE_IN_COLLISION)
        for label, joints in (('current state', None), ('seed', r['seed_joints'])):
            req = GetStateValidity.Request()
            req.group_name = r['group_name']
            req.robot_state.is_diff = True
            if joints is not None:
                req.robot_state.joint_state.name = list(r['joint_names'])
                req.robot_state.joint_state.position = [float(v) for v in joints]
            fut = self.moveit.validity.call_async(req)
            if not wait_future(fut, 10.0):
                self.get_logger().error(f'contacts at the {label}: no reply')
                continue
            pairs = sorted({' <-> '.join(sorted((c.contact_body_1, c.contact_body_2)))
                            for c in fut.result().contacts})
            self.get_logger().error(f'contacts at the {label}: {"; ".join(pairs) or "none"}')

    def _fb(self, msg: object) -> None:
        f = msg.feedback  # type: ignore[attr-defined]
        if f.phase != self.last_phase or f.last_rejection:
            self.last_phase = f.phase
            extra = f' (last rejection: {f.last_rejection})' if f.last_rejection else ''
            self.log(f'phase {f.phase}: {f.accepted} accepted, {f.rejected} rejected{extra}')

    def _truth(self, r_: Calibrate.Result) -> bool:
        truth = self.cfg[self.mode]
        X = pose(truth['true_camera_xyz_rpy']) @ tf.make(
            Rotation.from_euler('xyz', OPTICAL_RPY), np.zeros(3))
        t, q = r_.camera_transform.transform.translation, r_.camera_transform.transform.rotation
        got = tf.make(Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z]))
        dt = tf.translation_distance(got, X) * 1000
        dr = math.degrees(tf.rotation_angle(got, X))
        self.log(f'error against the true camera mount: {dt:.2f} mm, {dr:.3f} deg')
        return dt < 3.0 and dr < 0.5


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Runner()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    ex.shutdown(timeout_sec=2.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
