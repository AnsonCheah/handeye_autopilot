"""Test harness for the M3 node: a fake robot TF publisher, scripted poses that
show the board, and an in-process driver for the action and services.

The fake robot publishes ``base -> tool`` on /tf at a fixed rate with the
current node-clock stamp, like robot_state_publisher; ``freeze`` stops it (a
frozen joint_states), ``delay`` delivers each transform late (delayed TF).
"""

from __future__ import annotations

from collections import deque
import math
import threading
import time

from handeye_autopilot.core import handeye
from handeye_autopilot.core.render import look_at_board
from handeye_autopilot.ros import tf_tools
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from handeye_autopilot_interfaces.action import Calibrate


class FakeRobot(Node):
    def __init__(self, base='base_link', tool='tool0', rate=50.0, **kw):
        super().__init__('fake_robot', **kw)
        self.base, self.tool = base, tool
        self.G = np.eye(4)
        self.freeze = False
        self.delay = 0.0
        self._queue: deque = deque()
        self.history: deque = deque(maxlen=5000)      # (stamp_ns, G) as published
        self._lock = threading.Lock()
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.create_timer(1.0 / rate, self._tick)

    def move_to(self, G):
        with self._lock:
            self.G = np.array(G, dtype=float)

    def _tick(self):
        now = self.get_clock().now()
        if not self.freeze:
            with self._lock:
                msg = tf_tools.to_msg(self.G, self.base, self.tool, now.to_msg())
                self.history.append((now.nanoseconds, self.G.copy()))
            self._queue.append((time.monotonic() + self.delay, msg))
        while self._queue and self._queue[0][0] <= time.monotonic():
            self.pub.publish(TFMessage(transforms=[self._queue.popleft()[1]]))


def truth(camera_node):
    """X (parent_T_optical) and Z from a SyntheticCamera instance."""
    return camera_node.true_X, camera_node.Z


def scripted_poses(mode, X, Z, camera, board, n=12, seed=1):
    """Tool poses that put the board in good, varied views (tilts, rolls, depths)."""
    rng = np.random.default_rng(seed)
    w, h = board.size_m
    centre = np.array([w / 2, h / 2, 0.0])
    out = []
    for _ in range(n):
        tx, ty = rng.uniform(-30, 30, 2)
        if abs(tx) + abs(ty) < 15:                 # never near face-on (ambiguity)
            tx = math.copysign(20, tx)
        C = look_at_board(centre, rng.uniform(0.5, 0.65), math.radians(tx),
                          math.radians(ty), math.radians(rng.uniform(-40, 40)),
                          offset_px=tuple(rng.uniform(-40, 40, 2)), camera=camera)
        out.append(handeye.tool_pose_for(mode, X, Z, C))
    return out


def camera_params(mode, **over):
    """SyntheticCamera parameters for a mode with sensible true mounts."""
    if mode == 'eye_to_hand':
        p = {'mode': mode, 'true_camera_xyz_rpy': [1.0, 0.1, 0.6, 0.05, 0.4, math.pi - 0.1],
             'true_board_xyz_rpy': [0.01, -0.02, 0.06, 0.05, -0.03, 0.2]}
    else:
        p = {'mode': mode, 'true_camera_xyz_rpy': [0.05, -0.03, 0.08, 0.1, -0.05, 0.3],
             'true_board_xyz_rpy': [0.6, 0.1, 0.0, 0.02, 0.03, 0.4]}
    p.update(over)
    return p


class Driver:
    """Action + service clients for the calibration node, waits with deadlines."""

    def __init__(self, node: Node, ns='/handeye_autopilot'):
        self.node = node
        self.action = ActionClient(node, Calibrate, f'{ns}/calibrate')
        self.capture = node.create_client(Trigger, f'{ns}/capture')
        self.finish = node.create_client(Trigger, f'{ns}/finish')
        self.clear_fault = node.create_client(Trigger, f'{ns}/clear_fault')
        self.inject_fault = node.create_client(Trigger, f'{ns}/inject_fault')
        self.feedback: list = []
        self.debug = None          # optional callable describing the node's state

    def wait(self, fut, timeout=30.0):
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise TimeoutError('future not done')
            time.sleep(0.01)
        return fut.result()

    def send(self, mode='eye_to_hand', execution='manual', profile='', timeout=10.0):
        assert self.action.wait_for_server(timeout_sec=timeout)
        g = Calibrate.Goal(mode=mode, execution=execution, profile=profile)
        return self.wait(self.action.send_goal_async(
            g, feedback_callback=lambda f: self.feedback.append(f.feedback)), timeout)

    def call(self, client, timeout=10.0):
        assert client.wait_for_service(timeout_sec=timeout)
        return self.wait(client.call_async(Trigger.Request()), timeout)

    def phase(self):
        return self.feedback[-1].phase if self.feedback else ''

    def wait_phase(self, phase, timeout=20.0):
        t0 = time.monotonic()
        while self.phase() != phase:
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(f'phase {phase!r} not reached (at {self.phase()!r})')
            time.sleep(0.02)

    def capture_and_wait(self, timeout=45.0):
        """Trigger one capture; returns the feedback after it finished."""
        n = len(self.feedback)
        res = self.call(self.capture)
        assert res.success, res.message
        t0 = time.monotonic()
        while True:
            seen = False
            for f in list(self.feedback[n:]):
                if f.phase == Calibrate.Feedback.PHASE_CAPTURING:
                    seen = True
                elif seen:
                    return f
            if time.monotonic() - t0 > timeout:
                phases = [f.phase for f in self.feedback[n:]]
                raise TimeoutError(f'capture did not finish; feedback since: {phases}; '
                                   f'state: {self.debug() if self.debug else None}')
            time.sleep(0.02)


class SimClock(Node):
    """Publishes /clock from wall time (ROS sim time); ``jump(dt)`` shifts it."""

    def __init__(self, start=1000.0, rate=200.0):
        super().__init__('sim_clock')
        from rosgraph_msgs.msg import Clock
        self._Clock = Clock
        self.offset = start - time.monotonic()
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.create_timer(1.0 / rate, self._tick)

    def jump(self, dt):
        self.offset += dt

    def pause(self):
        """Freeze sim time where it is (a paused simulator)."""
        self.frozen = time.monotonic() + self.offset

    def _tick(self):
        t = getattr(self, 'frozen', None) or time.monotonic() + self.offset
        m = self._Clock()
        m.clock.sec = int(t)
        m.clock.nanosec = int((t - int(t)) * 1e9)
        self.pub.publish(m)
