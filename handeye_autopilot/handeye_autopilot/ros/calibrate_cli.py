"""Start a calibration from the terminal and follow it.

    ros2 run handeye_autopilot calibrate --mode eye_to_hand --execution moveit

Like ``ros2 action send_goal --feedback``, with what an operator needs on top:

- if the node does not answer the goal within ``--accept-timeout`` (DDS can
  drop the goal-accepted reply under load, seen in M4), it calls the node's
  ``~/cancel`` (the run may have started without us) and exits 2;
- Ctrl-C cancels the goal and waits for the result (MoveIt confirming the
  stop); without a result it calls ``~/cancel`` too, then exits 130;
- prints progress and the result, and exits 0 only if the run SUCCEEDED and
  passed every quality gate (1 otherwise, 3 if the goal was rejected; the
  node logs why).

ROS 2 concept: an action client
(https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html).
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from typing import Any

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from handeye_autopilot_interfaces.action import Calibrate

from .moveit_client import wait_future

CODES = {v: k for k, v in vars(Calibrate.Result).items()
         if k.isupper() and isinstance(v, int)}


def _stop_run(node: Node, ns: str, why: str) -> None:
    """Cancel whatever run the node has through ~/cancel; retried a few times,
    since a goal still being admitted has no run to cancel yet."""
    cli = node.create_client(Trigger, f'{ns}/cancel')
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f'{why}; {ns}/cancel is not available: check the node and the robot',
              file=sys.stderr)
        return
    for _attempt in range(3):
        fut = cli.call_async(Trigger.Request())
        if not wait_future(fut, 10.0):
            print(f'{why}; {ns}/cancel did not answer: check the node and the robot',
                  file=sys.stderr)
            return
        r = fut.result()
        print(f'{why}; {ns}/cancel: {r.message}', file=sys.stderr)
        if r.success:
            return
        time.sleep(2.0)


def run(args: argparse.Namespace, node: Node, interrupted: threading.Event) -> int:
    ns = args.node.rstrip('/')
    client = ActionClient(node, Calibrate, f'{ns}/calibrate')
    if not client.wait_for_server(timeout_sec=args.server_timeout):
        print(f'no action server {ns}/calibrate (is the node running?)', file=sys.stderr)
        return 1
    last = {'phase': '', 'detail': ''}

    def feedback(msg: Any) -> None:
        f = msg.feedback
        line = f'{f.phase}: {f.accepted} accepted, {f.rejected} rejected'
        if f.last_rejection:
            line += f' (last: {f.last_rejection})'
        if f.phase != last['phase'] or f.last_rejection_detail != last['detail']:
            print(line + (f' - {f.last_rejection_detail}' if f.last_rejection_detail and
                          f.last_rejection_detail != last['detail'] else ''), flush=True)
        last['phase'], last['detail'] = f.phase, f.last_rejection_detail
    goal = Calibrate.Goal(mode=args.mode, execution=args.execution, profile=args.profile)
    send = client.send_goal_async(goal, feedback_callback=feedback)
    if not wait_future(send, args.accept_timeout, interrupted):
        # the node may have accepted it and started (with motion) without us
        _stop_run(node, ns, 'no goal reply' if not interrupted.is_set() else 'interrupted')
        return 130 if interrupted.is_set() else 2
    handle = send.result()
    if not handle.accepted:
        print('goal rejected (busy, invalid mode/execution, unknown profile or a latched '
              'motion fault): see the node log', file=sys.stderr)
        return 3
    print(f'goal accepted ({args.mode}, {args.execution}); Ctrl-C cancels', flush=True)
    res_fut = handle.get_result_async()
    while not res_fut.done():
        if interrupted.is_set():
            print('cancelling (waiting for the robot to stop)...', file=sys.stderr, flush=True)
            handle.cancel_goal_async()
            if not wait_future(res_fut, args.cancel_timeout):
                _stop_run(node, ns, 'no result after the cancel')
                return 130
            break
        time.sleep(0.05)
    res = res_fut.result()
    r = res.result
    q = r.quality
    status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
              GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(res.status, str(res.status))
    print(f'{status} ({CODES.get(r.error_code, r.error_code)}): {r.message}')
    if r.transform_valid:
        t = r.camera_transform.transform.translation
        print(f'camera {r.camera_transform.header.frame_id} -> '
              f'{r.camera_transform.child_frame_id}: '
              f'[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}] m; gates passed: {r.passed}')
    if not math.isnan(q.reprojection_rms_px):
        print(f'captures {q.captures_accepted} accepted / {q.captures_rejected} rejected; '
              f'reprojection RMS {q.reprojection_rms_px:.2f} px, held-out '
              f'{q.heldout_reprojection_rms_px:.2f} px')
    if r.result_file:
        print(f'result: {r.result_file}')
    if interrupted.is_set():
        return 130
    ok = res.status == GoalStatus.STATUS_SUCCEEDED and r.error_code == Calibrate.Result.NONE \
        and r.passed
    return 0 if ok else 1


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--mode', required=True, choices=['eye_to_hand', 'eye_in_hand'])
    p.add_argument('--execution', default='moveit', choices=['moveit', 'manual'])
    p.add_argument('--profile', default='')
    p.add_argument('--node', default='/handeye_autopilot', help='the calibration node')
    p.add_argument('--server-timeout', type=float, default=30.0)
    p.add_argument('--accept-timeout', type=float, default=10.0)
    p.add_argument('--cancel-timeout', type=float, default=30.0)
    args, ros_args = p.parse_known_args(argv)
    # our own SIGINT handling: the context must stay up to send the cancel
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('handeye_calibrate_cli')
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    interrupted = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: interrupted.set())
    signal.signal(signal.SIGTERM, lambda *_: interrupted.set())
    try:
        code = run(args, node, interrupted)
    finally:
        ex.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
