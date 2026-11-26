"""M2 probe (03-milestones M2, "probe ahead"): what MoveIt really reports.

Throwaway script, not part of any package. It drives the Panda mock-hardware
demo through pymoveit2 and calls compute_ik directly, then writes what it saw
(terminal states, error codes, timings) to a YAML file that feeds M3/M4.

Run (two terminals, ROS sourced):
  ros2 launch moveit_resources_panda_moveit_config demo.launch.py
  python3 tools/probes/m2_moveit_probe.py -o probe_$ROS_DISTRO.yaml

Needs: ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-resources-panda-moveit-config
       ros-$ROS_DISTRO-pymoveit2, and the controllers the Panda demo spawns but
       does not depend on: ros-$ROS_DISTRO-joint-state-broadcaster
       ros-$ROS_DISTRO-joint-trajectory-controller
"""

from __future__ import annotations

import argparse
import os
import threading
import time

from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from pymoveit2 import MoveIt2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import yaml

JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
FAR = [1.2, 0.3, -0.4, -1.6, 0.5, 2.2, -0.3]
OUT_OF_BOUNDS = [0.0, -0.785, 0.0, 1.0, 0.0, 1.571, 0.785]   # joint4 URDF limit [-3.1416, 0.0873]
IN_COLLISION = [0.0] * 7                                       # hand hits link5 (seen on Jazzy)


def _wait(pred, timeout: float) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.01)
    return False


MOVED_RAD = 0.02


def _joints(m: MoveIt2) -> list[float] | None:
    js = m.joint_state
    if js is None:
        return None
    pos = dict(zip(js.name, js.position))
    return [pos[j] for j in JOINTS] if all(j in pos for j in JOINTS) else None


def _moved(m: MoveIt2, start: list[float]) -> float:
    now = _joints(m)
    return 0.0 if now is None else max(abs(a - b) for a, b in zip(now, start))


def motion(m: MoveIt2, goal: list[float], cancel: bool = False) -> dict:
    """One joint-goal motion; records what MoveGroup itself reported.

    accepted: the action server accepted the goal (goal handle);
    status: the action's terminal GoalStatus (4 SUCCEEDED, 5 CANCELED, 6 ABORTED);
    error_code: MoveIt's error code in the MoveGroup result (1 = SUCCESS).
    With ``cancel``, the cancel is sent only after execution is observed.
    """
    start = _joints(m)
    t0 = time.time()
    out_submit = m.move_to_configuration(goal, JOINTS, timeout_sec=10.0)
    out: dict = {'submitted': bool(out_submit)}
    left_requesting = _wait(lambda: str(m.query_state()) != 'MoveIt2State.REQUESTING', 10.0)
    out['left_requesting_s'] = round(time.time() - t0, 3) if left_requesting else None
    gf = m.get_goal_future()
    handle = gf.result() if gf is not None and gf.done() else None
    out['accepted'] = None if handle is None else bool(handle.accepted)
    rf = m.get_execution_future() if out['accepted'] else None
    t_cancel = None
    if cancel:
        # MoveGroup reports EXECUTING (goal accepted) while it is still planning,
        # so "under way" means the measured joints have actually started moving.
        executing = _wait(lambda: str(m.query_state()) == 'MoveIt2State.EXECUTING', 5.0)
        out['executing_seen'] = executing
        moving = executing and start is not None and _wait(
            lambda: _moved(m, start) > MOVED_RAD, 10.0)
        out['motion_seen'] = moving
        if moving:               # cancel only a motion that is really under way
            out['moved_at_cancel_rad'] = round(_moved(m, start), 4)
            t_cancel = time.time()
            out['cancel_request_returned'] = m.cancel_execution()
    ok = m.wait_until_executed(timeout_sec=20.0)
    out['wait_returned'] = ok
    out['seconds'] = round(time.time() - t0, 3)
    if t_cancel is not None:
        out['cancel_to_terminal_s'] = round(time.time() - t_cancel, 3)
    if rf is not None and _wait(rf.done, 5.0):
        res = rf.result()
        out['status'] = int(res.status)
        out['error_code'] = int(res.result.error_code.val)
    else:
        out['status'] = None
        code = m.get_last_execution_error_code()
        out['error_code'] = None if code is None else int(code.val)
    op = m._MoveIt2__lifecycle.last_operation   # probe only: raw GoalStatus kept by pymoveit2
    out['lifecycle_status'] = None if op is None or op.status is None else int(op.status)
    if t_cancel is not None:
        out['moved_after_cancel_rad'] = round(_moved(m, start), 4)
    out['last_failure'] = m.last_failure()
    time.sleep(0.5)                                  # let /joint_states catch up
    end = _joints(m)
    out['joints_after'] = None if end is None else [round(v, 4) for v in end]
    out['state_after'] = str(m.query_state())
    return out


def ik_call(node, client, pose, timeout: Duration) -> dict:
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'panda_arm'
    req.ik_request.ik_link_name = 'panda_link8'
    req.ik_request.robot_state.is_diff = True          # monitored scene state (01-arch §3.1)
    req.ik_request.avoid_collisions = True
    req.ik_request.pose_stamped = pose
    req.ik_request.timeout = timeout
    t0 = time.time()
    fut = client.call_async(req)
    while not fut.done() and time.time() - t0 < 15:
        time.sleep(0.01)
    if not fut.done():
        return {'done': False, 'seconds': round(time.time() - t0, 3)}
    res = fut.result()
    return {'done': True, 'seconds': round(time.time() - t0, 3),
            'error_code': int(res.error_code.val),
            'joints': [round(v, 4) for v in res.solution.joint_state.position[:7]]}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--output', default='probe.yaml')
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('m2_moveit_probe')
    cb = ReentrantCallbackGroup()
    m = MoveIt2(node, JOINTS, 'panda_link0', 'panda_link8', 'panda_arm',
                callback_group=cb, use_move_group_action=True)
    m.max_velocity = 0.3
    m.max_acceleration = 0.3
    ik = node.create_client(GetPositionIK, 'compute_ik', callback_group=cb)
    ex = MultiThreadedExecutor(4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(2.0)
    # Without joint states MoveIt assumes all-zero joints (in self-collision on
    # the Panda) and every motion aborts with START_STATE_IN_COLLISION, which
    # would be recorded as if it were a real outcome. Refuse to probe instead.
    t0 = time.time()
    while _joints(m) is None and time.time() - t0 < 10.0:
        time.sleep(0.1)
    if _joints(m) is None:
        ex.shutdown()
        rclpy.shutdown()
        raise SystemExit('no /joint_states from the demo: is joint_state_broadcaster '
                         'installed and active? (ros-$ROS_DISTRO-joint-state-broadcaster, '
                         'ros-$ROS_DISTRO-joint-trajectory-controller)')

    found = {'ros_distro': os.environ.get('ROS_DISTRO'), 'time': time.ctime()}
    found['to_ready'] = motion(m, READY)            # known start; may be a no-op
    found['success'] = motion(m, FAR)               # a real motion
    found['planning_failure'] = motion(m, IN_COLLISION)
    found['out_of_bounds_goal'] = motion(m, OUT_OF_BOUNDS)   # recorded, not judged
    found['back_to_far'] = motion(m, FAR)
    found['cancel_mid_motion'] = motion(m, READY, cancel=True)
    found['back_to_ready'] = motion(m, READY)
    fk = m.compute_fk()
    if fk is not None and ik.wait_for_service(timeout_sec=5.0):
        pose = fk[0] if isinstance(fk, list) else fk
        found['ik_whole_seconds'] = ik_call(node, ik, pose, Duration(sec=1))
        found['ik_nanosec_only'] = ik_call(node, ik, pose, Duration(sec=0, nanosec=500_000_000))
        far = type(pose)()
        far.header = pose.header
        far.pose.position.x = 3.0                      # unreachable
        far.pose.orientation.w = 1.0
        found['ik_unreachable'] = ik_call(node, ik, far, Duration(sec=1))
        # Does the server honour a nanosec-only timeout? Unreachable means the
        # solver uses all of it: ~0.5 s if honoured, near 0 (or server default) if dropped.
        found['ik_unreachable_nanosec_only'] = ik_call(
            node, ik, far, Duration(sec=0, nanosec=500_000_000))
    else:
        found['ik'] = 'compute_ik or FK unavailable'
    missing = [k for k in ('success', 'planning_failure', 'cancel_mid_motion')
               if found[k].get('status') is None and found[k].get('error_code') is None]
    if found['success'].get('status') != 4:
        missing.append('success.status_succeeded')
    if found['planning_failure'].get('status') == 4:
        missing.append('planning_failure.did_not_fail')
    if not found['cancel_mid_motion'].get('motion_seen'):
        missing.append('cancel_mid_motion.motion_seen')
    if not isinstance(found.get('ik_whole_seconds'), dict) or \
            not found['ik_whole_seconds'].get('done'):
        missing.append('ik_whole_seconds')
    found['inconclusive'] = missing
    with open(a.output, 'w') as f:
        yaml.safe_dump(found, f, sort_keys=False)
    print(yaml.safe_dump(found, sort_keys=False))
    ex.shutdown()
    rclpy.shutdown()
    if missing:
        raise SystemExit(f'inconclusive probe, missing: {missing}')


if __name__ == '__main__':
    main()
