"""Start a calibration from the terminal and follow it.

    ros2 run handeye_autopilot calibrate --mode eye_to_hand --execution moveit

Like ``ros2 action send_goal --feedback``, with what an operator needs on top:

- the goal carries an ID chosen here, so it can always be cancelled by that
  ID through the action's own cancel service, even without the goal handle,
  and never another operator's run;
- if the node does not answer the goal within ``--accept-timeout`` (DDS can
  drop the goal-accepted reply under load, seen in M4), it cancels that goal
  ID (the run may have started without us), waits for its result, exits 2;
- Ctrl-C cancels the goal and waits for its result (MoveIt confirming the
  stop), then exits 130; Ctrl-C before the goal is sent sends nothing;
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
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from unique_identifier_msgs.msg import UUID

from handeye_autopilot_interfaces.action import Calibrate

from .moveit_client import wait_future

CODES = {v: k for k, v in vars(Calibrate.Result).items()
         if k.isupper() and isinstance(v, int)}


def _cancel_by_id(client: ActionClient, goal_id: UUID, timeout: float, why: str) -> None:
    """Cancel OUR goal by its ID, without the goal handle the lost reply would
    have given us: a handle for that ID on the same action client, so the cancel
    and result requests use the client's own (remapped) action channels.
    Retried while the node may still be admitting the goal; then wait for its
    result, which says whether the stop was confirmed."""
    handle = ClientGoalHandle(client, goal_id,
                              Calibrate.Impl.SendGoalService.Response(accepted=True))
    for _attempt in range(3):
        fut = handle.cancel_goal_async()
        if not wait_future(fut, 10.0):
            print(f'{why}: no answer to the cancel request: CHECK THE ROBOT',
                  file=sys.stderr)
            return
        code = fut.result().return_code
        if code in (CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED):
            if code == CancelGoal.Response.ERROR_NONE:
                print(f'{why}: cancelling our goal; waiting for it to stop...',
                      file=sys.stderr, flush=True)
            else:
                print(f'{why}: our goal had already ended', file=sys.stderr)
            got = handle.get_result_async()
            if not wait_future(got, timeout):
                print('no result for our goal: CHECK THE ROBOT', file=sys.stderr)
                return
            res = got.result()
            r = res.result
            if res.status == GoalStatus.STATUS_CANCELED or \
                    r.error_code == Calibrate.Result.CANCELED:
                print(f'stopped: {r.message}', file=sys.stderr)
            elif r.error_code == Calibrate.Result.MOTION_FAULT or \
                    code == CancelGoal.Response.ERROR_NONE:
                print(f'the goal ended {CODES.get(r.error_code, r.error_code)}: {r.message} '
                      '(the stop is NOT confirmed: CHECK THE ROBOT)', file=sys.stderr)
            else:
                print(f'it ended {CODES.get(r.error_code, r.error_code)}: {r.message}',
                      file=sys.stderr)
            return
        time.sleep(2.0)                    # unknown yet: may still be being admitted
    print(f'{why}: the node does not know our goal (never accepted, or ended)',
          file=sys.stderr)


def run(args: argparse.Namespace, node: Node, interrupted: threading.Event) -> int:
    ns = args.node.rstrip('/')
    client = ActionClient(node, Calibrate, f'{ns}/calibrate')
    deadline = time.monotonic() + args.server_timeout
    while not client.wait_for_server(timeout_sec=0.2):      # Ctrl-C stays prompt
        if interrupted.is_set():
            return 130
        if time.monotonic() > deadline:
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
    # discovery is per direction: the server can see our request before it has
    # matched our reply reader, and a reply sent then is lost (seen under load)
    if interrupted.wait(1.0):
        return 130                          # nothing sent
    goal = Calibrate.Goal(mode=args.mode, execution=args.execution, profile=args.profile)
    goal_id = UUID(uuid=list(uuid.uuid4().bytes))
    send = client.send_goal_async(goal, feedback_callback=feedback, goal_uuid=goal_id)
    if not wait_future(send, args.accept_timeout, interrupted):
        # the node may have accepted it and started (with motion) without us
        _cancel_by_id(client, goal_id, args.cancel_timeout,
                      'no goal reply' if not interrupted.is_set() else 'interrupted')
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
                print('no result after the cancel: check the robot', file=sys.stderr)
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
