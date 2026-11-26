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
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
import numpy as np
from pymoveit2 import MoveIt2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation
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
    pos = dict(zip(js.name, js.position, strict=True))
    return [pos[j] for j in JOINTS] if all(j in pos for j in JOINTS) else None


def _moved(m: MoveIt2, start: list[float]) -> float:
    now = _joints(m)
    return 0.0 if now is None else max(abs(a - b) for a, b in zip(now, start, strict=True))


def motion(m: MoveIt2, goal: list[float] | None, cancel: bool = False,
           submit=None) -> dict:
    """One joint-goal motion; records what MoveGroup itself reported.

    accepted: the action server accepted the goal (goal handle);
    status: the action's terminal GoalStatus (4 SUCCEEDED, 5 CANCELED, 6 ABORTED);
    error_code: MoveIt's error code in the MoveGroup result (1 = SUCCESS).
    With ``cancel``, the cancel is sent only after execution is observed.
    """
    start = _joints(m)
    t0 = time.time()
    out_submit = submit() if submit is not None else \
        m.move_to_configuration(goal, JOINTS, timeout_sec=10.0)
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
        # evidence of whether execution began (01-architecture §3.2 not_started rule)
        # (MoveGroup results only; a Cartesian move runs ExecuteTrajectory, which has neither)
        out['action'] = type(res.result).__name__
        for key, field in (('planned_points', 'planned_trajectory'),
                           ('executed_points', 'executed_trajectory')):
            traj = getattr(res.result, field, None)
            out[key] = None if traj is None else len(traj.joint_trajectory.points)
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


def _mat(pose) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w]).as_matrix()
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def _pose(T: np.ndarray) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def _call(client, req, timeout: float = 15.0):
    fut = client.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.01)
    return fut.result() if fut.done() else None


def offset_tcp_path(cart, fk, link8, offset: float, degrees: float, dense: bool) -> dict:
    """01-architecture §3.2: a rotation about a TCP ``offset`` m along link8 z
    (not in the URDF), planned with GetCartesianPath for link8 either from the
    end pose alone or through dense waypoints converted from the TCP path.
    Measures how far the TCP strays on the planned trajectory (ideal: 0)."""
    off = np.eye(4)
    off[2, 3] = offset
    tcp0 = link8 @ off

    def link_for(deg):
        rot = np.eye(4)
        rot[:3, :3] = Rotation.from_euler('x', deg, degrees=True).as_matrix()
        return tcp0 @ rot @ np.linalg.inv(off)
    steps = [degrees] if not dense else list(np.linspace(1.0, degrees, int(degrees)))
    req = GetCartesianPath.Request()
    req.header.frame_id = 'panda_link0'
    req.start_state.is_diff = True
    req.group_name, req.link_name = 'panda_arm', 'panda_link8'
    req.waypoints = [_pose(link_for(d)) for d in steps]
    req.max_step, req.avoid_collisions = 0.0025, True
    res = _call(cart, req)
    if res is None:
        return {'done': False}
    pts = res.solution.joint_trajectory.points
    names = res.solution.joint_trajectory.joint_names
    worst = 0.0
    last = None
    for pt in pts:
        f = GetPositionFK.Request()
        f.header.frame_id = 'panda_link0'
        f.fk_link_names = ['panda_link8']
        f.robot_state.joint_state.name = list(names)
        f.robot_state.joint_state.position = list(pt.positions)
        r = _call(fk, f)
        if r is None or not r.pose_stamped:
            return {'done': False, 'fk_failed': True}
        tcp = _mat(r.pose_stamped[0].pose) @ off
        worst = max(worst, float(np.linalg.norm(tcp[:3, 3] - tcp0[:3, 3])))
        last = tcp
    return {'done': True, 'waypoints': len(steps), 'fraction': float(res.fraction),
            'points': len(pts), 'error_code': int(res.error_code.val),
            'tcp_max_deviation_mm': round(worst * 1000, 3),
            # the trajectory really performs the rotation (not a stationary path)
            'tcp_rotation_deg': None if last is None else round(float(np.degrees(
                Rotation.from_matrix(tcp0[:3, :3].T @ last[:3, :3]).magnitude())), 3),
            # and it is the requested rotation (axis included): angle to the goal
            'tcp_goal_error_deg': None if last is None else round(float(np.degrees(
                Rotation.from_matrix((tcp0 @ _rx(degrees))[:3, :3].T @ last[:3, :3])
                .magnitude())), 3)}


def _rx(deg: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('x', deg, degrees=True).as_matrix()
    return T


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
    # Cartesian (straight tool path) moves, as used for checked short moves
    # (01-architecture §3.2): a reachable 5 cm shift and an impossible 3 m one,
    # both requiring the whole path (fraction threshold 1.0).
    fk0 = m.compute_fk()
    if fk0 is not None:
        p0 = fk0[0] if isinstance(fk0, list) else fk0
        q = p0.pose.orientation
        quat = (q.x, q.y, q.z, q.w)

        def cart(dx: float):
            pos = (p0.pose.position.x + dx, p0.pose.position.y, p0.pose.position.z)
            return lambda: m.move_to_pose(position=pos, quat_xyzw=quat, frame_id='panda_link0',
                                          cartesian=True, cartesian_fraction_threshold=1.0,
                                          timeout_sec=10.0)
        found['cartesian_ok'] = motion(m, None, submit=cart(0.05))
        # where the tool really ended (FK of the measured joints) vs the request
        fk1 = m.compute_fk()
        if fk1 is not None:
            p1 = (fk1[0] if isinstance(fk1, list) else fk1).pose
            want = np.array([p0.pose.position.x + 0.05, p0.pose.position.y,
                             p0.pose.position.z])
            got = np.array([p1.position.x, p1.position.y, p1.position.z])
            dq = Rotation.from_quat(quat).inv() * Rotation.from_quat(
                [p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w])
            found['cartesian_ok']['end_position_error_mm'] = round(
                float(np.linalg.norm(got - want)) * 1000, 3)
            found['cartesian_ok']['end_rotation_error_deg'] = round(
                float(np.degrees(dq.magnitude())), 3)
        found['cartesian_back'] = motion(m, None, submit=cart(0.0))
        found['cartesian_impossible'] = motion(m, None, submit=cart(3.0))
        cart_srv = node.create_client(GetCartesianPath, 'compute_cartesian_path',
                                      callback_group=cb)
        fk_srv = node.create_client(GetPositionFK, 'compute_fk', callback_group=cb)
        if cart_srv.wait_for_service(timeout_sec=5.0) and fk_srv.wait_for_service(timeout_sec=5.0):
            link8 = _mat(p0.pose)
            found['offset_tcp_endpoint_only'] = offset_tcp_path(cart_srv, fk_srv, link8,
                                                                0.3, 20.0, dense=False)
            found['offset_tcp_dense_waypoints'] = offset_tcp_path(cart_srv, fk_srv, link8,
                                                                  0.3, 20.0, dense=True)
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
    got = found['success'].get('joints_after')
    if got is None or max(abs(a - b) for a, b in zip(got, FAR, strict=True)) > 0.01:
        missing.append('success.did_not_reach_goal')
    if found['planning_failure'].get('status') == 4:
        missing.append('planning_failure.did_not_fail')
    # the not_started rule rests on these (01-architecture §3.2)
    if found['planning_failure'].get('status') != 6:          # ABORTED, not CANCELED
        missing.append('planning_failure.not_aborted')
    if found['planning_failure'].get('planned_points') != 0:
        missing.append('planning_failure.planned_points_not_zero')
    # ... and the arm did not move (the not_started evidence)
    before = found['success'].get('joints_after')
    after = found['planning_failure'].get('joints_after')
    if before is None or after is None or \
            max(abs(a - b) for a, b in zip(before, after, strict=True)) > 0.01:
        missing.append('planning_failure.arm_moved')
    if not found['cancel_mid_motion'].get('planned_points'):
        missing.append('cancel_mid_motion.planned_points_missing')
    if found.get('cartesian_ok', {}).get('status') != 4:
        missing.append('cartesian_ok.not_succeeded')
    # consistent evidence: SUCCEEDED goes with SUCCESS, a refused path leaves the arm still
    for k, v in found.items():
        if isinstance(v, dict) and v.get('status') == 4 and v.get('error_code') != 1:
            missing.append(f'{k}.succeeded_without_success_code')
    jb = found.get('cartesian_back', {}).get('joints_after')
    ji = found.get('cartesian_impossible', {}).get('joints_after')
    if jb is None or ji is None or max(abs(a - b) for a, b in zip(jb, ji, strict=True)) > 0.01:
        missing.append('cartesian_impossible.arm_moved')
    co = found.get('cartesian_ok', {})
    if co.get('end_position_error_mm') is None or co['end_position_error_mm'] > 1.0 or \
            co.get('end_rotation_error_deg') is None or co['end_rotation_error_deg'] > 0.1:
        missing.append('cartesian_ok.end_pose_not_reached')
    j0 = found['back_to_ready'].get('joints_after')
    j1 = found.get('cartesian_ok', {}).get('joints_after')
    if j0 is None or j1 is None or max(abs(a - b) for a, b in zip(j0, j1, strict=True)) < 0.01:
        missing.append('cartesian_ok.did_not_move')
    # an impossible Cartesian path must be refused before anything is sent
    ci = found.get('cartesian_impossible', {})
    if ci.get('submitted') is not False or 'reached only' not in str(ci.get('last_failure')):
        missing.append('cartesian_impossible.not_refused_for_path_fraction')
    for k in ('offset_tcp_endpoint_only', 'offset_tcp_dense_waypoints'):
        r = found.get(k, {})
        if r.get('fraction') != 1.0 or r.get('error_code') != 1 or \
                (r.get('points') or 0) < 2 or \
                r.get('tcp_max_deviation_mm') is None or \
                abs((r.get('tcp_rotation_deg') or 0.0) - 20.0) > 0.5 or \
                r.get('tcp_goal_error_deg') is None or r['tcp_goal_error_deg'] > 0.5:
            missing.append(k)
    # the evidence itself: dense waypoints keep the TCP on its path (sub-mm) and
    # clearly better than the end pose alone
    d = found.get('offset_tcp_dense_waypoints', {}).get('tcp_max_deviation_mm')
    e = found.get('offset_tcp_endpoint_only', {}).get('tcp_max_deviation_mm')
    if d is None or e is None or not (d < 0.5 and d < e / 10):
        missing.append('offset_tcp.dense_not_better')
    # the cancel must be accepted and end the goal as CANCELED (not some other abort)
    cm = found['cancel_mid_motion']
    if cm.get('cancel_request_returned') is not True or cm.get('status') != 5:
        missing.append('cancel_mid_motion.not_canceled')
    for k in ('ik_whole_seconds', 'ik_nanosec_only'):
        if isinstance(found.get(k), dict) and found[k].get('error_code') != 1:
            missing.append(f'{k}.reachable_pose_failed')
    if not found['cancel_mid_motion'].get('motion_seen'):
        missing.append('cancel_mid_motion.motion_seen')
    if not isinstance(found.get('ik_whole_seconds'), dict) or \
            not found['ik_whole_seconds'].get('done'):
        missing.append('ik_whole_seconds')
    for k in ('ik_unreachable', 'ik_unreachable_nanosec_only'):
        if not (isinstance(found.get(k), dict) and found[k].get('done') and
                found[k].get('error_code') == -31):
            missing.append(f'{k}.no_completed_no_solution_reply')
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
