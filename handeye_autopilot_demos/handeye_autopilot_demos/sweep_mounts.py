"""Run the demo calibration for many board / camera mounts (config/mount_sweep.yaml).

    ros2 run handeye_autopilot_demos sweep_mounts --robots panda ur --jobs 4

For each robot it reads the tool pose at the seed joints from MoveIt (FK),
builds one demo config per case and mode (mounts.case_config), runs
``demo.launch.py`` headless for each, several at a time, and writes
``results.yaml`` and ``results.md`` to ``--out``. Each run gets its own ROS
domain, so parallel runs never see each other's topics
(https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html).

Verdicts: ``passed`` (every quality gate and within the demo's 3 mm / 0.5 deg
of the true mount), ``failed`` (the calibration ran and failed, or was off),
``setup`` (the robot could not reach the seed: the moved board or camera
collides there, so nobody could start a calibration with that mount).
Exit code 0 when no case failed.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
import yaml

from handeye_autopilot_demos import mounts

MODES = ('eye_to_hand', 'eye_in_hand')
ERR = re.compile(r'error against the true camera mount: ([\d.]+) mm, ([\d.]+) deg')
FINISHED = re.compile(r'finished: status (\d+), error_code (\d+): (.*)')
ENDED = re.compile(r'the run ended: status (\d+), error_code (\d+): (.*)')  # after a cancel
CAPTURES = re.compile(r'captures accepted (\d+), rejected (\d+)')
# the runner's seed-move failure with collision evidence from MoveIt
SEED_COLLISION = re.compile(r'could not reach the seed: status \d+, '
                            r'(START_STATE_IN_COLLISION|GOAL_IN_COLLISION)')
CONTACTS = re.compile(r'contacts at the (current state|seed): (.*)')
MOUNTED = ('calibration_board', 'camera')   # scene.py's ids for the board and the camera
# which probe's contacts explain MoveIt's code (run_demo._log_contacts)
FAILED_STATE = {'START_STATE_IN_COLLISION': 'current state', 'GOAL_IN_COLLISION': 'seed'}
STOP = threading.Event()                 # Ctrl-C: launch nothing more
_running: set[subprocess.Popen] = set()  # launches to stop on Ctrl-C
_running_lock = threading.Lock()


def seed_tool_pose(robot: str, cfg: dict[str, Any], domain: int) -> Any:
    """base -> tool at the seed joints, from the robot's own MoveIt (compute_fk)."""
    from handeye_autopilot.ros.moveit_client import MoveItClient
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor

    r = cfg['robot']
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    ctx = rclpy.Context()
    proc = None
    try:
        # Ctrl-C held off while the child starts, so proc is set whenever it runs
        signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
        try:
            proc = subprocess.Popen(['ros2', 'launch', 'handeye_autopilot_demos', r['launch']],
                                    env=env, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, start_new_session=True,
                                    preexec_fn=_unblock_sigint)   # the mask is inherited
        finally:
            signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT})
        rclpy.init(context=ctx, domain_id=domain)
        node = rclpy.create_node('sweep_fk', context=ctx)
        ex = MultiThreadedExecutor(num_threads=2, context=ctx)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        client = MoveItClient(node, ReentrantCallbackGroup())
        if client.missing(120.0):
            raise RuntimeError(f'{robot}: MoveIt did not start')
        out = None
        for _ in range(10):          # right after start-up FK may not answer yet
            out = client.fk_poses(r['joint_names'], [r['seed_joints']], r['tool_frame'],
                                  r['base_frame'], 10.0)
            if out:
                break
            time.sleep(2.0)
        if not out:
            raise RuntimeError(f'{robot}: compute_fk failed at the seed joints')
        ex.shutdown(timeout_sec=2.0)
        return out[0]
    finally:
        if ctx.ok():
            rclpy.shutdown(context=ctx)
        if proc is not None:
            _stop(proc)


def _unblock_sigint() -> None:
    signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT})


def _stop(proc: subprocess.Popen) -> None:
    """Stop a launch and everything it started (its process group)."""
    try:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    except ProcessLookupError:          # it ended on its own meanwhile
        proc.wait()


def run_case(job: dict[str, Any], domains: queue.Queue, timeout: float) -> dict[str, Any]:
    domain = domains.get()
    t0 = time.time()
    try:
        log = Path(job['dir']) / 'launch.log'
        with open(log, 'w') as f, _running_lock:
            if STOP.is_set():
                return dict(job, verdict='failed', message='sweep interrupted', minutes=0.0)
            proc = subprocess.Popen(
                ['ros2', 'launch', 'handeye_autopilot_demos', 'demo.launch.py',
                 f"robot:={job['robot']}", f"mode:={job['mode']}",
                 f"config:={job['config']}", 'stay:=false'],
                env=dict(os.environ, ROS_DOMAIN_ID=str(domain)), stdout=f,
                stderr=subprocess.STDOUT, start_new_session=True)
            _running.add(proc)
        try:
            code = proc.wait(timeout)
        except subprocess.TimeoutExpired:
            code = None
        _stop(proc)
        with _running_lock:
            _running.discard(proc)
        if STOP.is_set():
            return dict(job, verdict='failed', message='sweep interrupted', minutes=0.0)
        return dict(job, **parse(log.read_text(), code), minutes=round((time.time() - t0) / 60, 1))
    finally:
        domains.put(domain)


def parse(text: str, code: int | None) -> dict[str, Any]:
    """The verdict and numbers from a demo launch's output."""
    out: dict[str, Any] = {'exit_code': code}
    if m := ERR.search(text):
        out['error_mm'], out['error_deg'] = float(m.group(1)), float(m.group(2))
    if m := CAPTURES.search(text):
        out['accepted'], out['rejected'] = int(m.group(1)), int(m.group(2))
    if m := FINISHED.search(text) or ENDED.search(text):
        out['message'] = m.group(m.lastindex).strip()
    if code == 0 and 'DEMO PASSED' in text:
        out['verdict'] = 'passed'
    elif code is not None and (m := SEED_COLLISION.search(text)) and (
            pairs := [p for c in CONTACTS.finditer(text)
                      if c.group(1) == FAILED_STATE[m.group(1)]
                      for p in c.group(2).split('; ')
                      if any(o in p.split(' <-> ') for o in MOUNTED)]):
        # MoveIt's collision verdict on the robot's start or seed state, with the
        # board or the camera as one of the colliding bodies: nobody could start a
        # calibration with this mount
        out['verdict'] = 'setup'
        out['message'] = (f'the mount collides before calibrating ({m.group(1)}: '
                          f'{"; ".join(sorted(set(pairs)))})')
    else:
        out['verdict'] = 'failed'
        if code is None:
            out['message'] = 'timed out'
        out.setdefault('message', 'no result (see launch.log)')
    return out


def table(rows: list[dict[str, Any]]) -> str:
    lines = ['| Robot | Mode | Case | Offset (mm; deg) | lever_max | Verdict | Error vs truth '
             '| Captures | Minutes | Notes |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        off = ', '.join(f'{1000 * v:.0f}' for v in r['translation_m']) + '; ' + ', '.join(
            f'{v:.0f}' for v in r['rotation_deg'])
        err = (f"{r['error_mm']:.2f} mm, {r['error_deg']:.3f}°" if 'error_mm' in r else '')
        caps = f"{r['accepted']} / {r['rejected']}" if 'accepted' in r else ''
        note = '; '.join(r['notes'] + ([r['message']] if r['verdict'] != 'passed' else []))
        lines.append(f"| {r['robot']} | {r['mode']} | {r['name']} | {off} | "
                     f"{r['lever_max']:.2f} | {r['verdict']} | {err} | {caps} | "
                     f"{r['minutes']} | {note} |")
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    share = Path(get_package_share_directory('handeye_autopilot_demos'))
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--robots', nargs='+', default=['panda', 'ur'])
    ap.add_argument('--modes', nargs='+', default=list(MODES), choices=MODES)
    ap.add_argument('--cases', nargs='*', help='case names (default: all)')
    ap.add_argument('--spec', default=str(share / 'config' / 'mount_sweep.yaml'))
    ap.add_argument('--jobs', type=int, default=3, help='runs at the same time')
    ap.add_argument('--timeout', type=float, default=1800.0,
                    help='s the runner waits for the calibration result; each launch '
                         'also gets 1500 s for start-up, the seed move and stopping')
    ap.add_argument('--domain-base', type=int, default=120,
                    help='first ROS domain ID used (one per parallel run, plus one for '
                         'FK); two sweeps at once need ranges that do not overlap')
    ap.add_argument('--out', default=os.path.expanduser(
        '~/.ros/handeye_autopilot/mount_sweep/' + time.strftime('%Y%m%dT%H%M%S')))
    a = ap.parse_args(argv)
    spec = yaml.safe_load(Path(a.spec).read_text())
    known = mounts.cases(spec)
    unknown = sorted(set(a.cases or []) - {c['name'] for c in known})
    if unknown:
        ap.error(f"unknown case(s) {', '.join(unknown)}; known: "
                 f"{', '.join(c['name'] for c in known)}")
    selected = [c for c in known if not a.cases or c['name'] in a.cases]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    jobs = []
    for robot in a.robots:
        cfg = yaml.safe_load((share / 'config' / f'{robot}.yaml').read_text())
        print(f'{robot}: reading the seed tool pose from MoveIt...', flush=True)
        G = seed_tool_pose(robot, cfg, a.domain_base)
        for mode in a.modes:
            for case in selected:
                d = out / f"{robot}_{mode}_{case['name']}"
                d.mkdir(exist_ok=True)
                c = mounts.case_config(cfg, mode, G, case)
                c['calibration']['output_dir'] = str(d)
                c['demo_timeout'] = a.timeout
                (d / 'config.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
                jobs.append(dict(robot=robot, mode=mode, dir=str(d), config=str(d / 'config.yaml'),
                                 lever_max=c['calibration']['session']['lever_max'],
                                 notes=mounts.notes(c, mode), **case))
    print(f'{len(jobs)} runs, {a.jobs} at a time; logs in {out}', flush=True)
    domains: queue.Queue = queue.Queue()
    for i in range(a.jobs):
        domains.put(a.domain_base + 1 + i)
    rows: list[dict[str, Any]] = []
    pool = ThreadPoolExecutor(a.jobs)
    try:
        # the runner's own deadline is --timeout for the calibration; start-up,
        # discovery and the seed move come on top
        futures = [pool.submit(run_case, j, domains, a.timeout + 1500) for j in jobs]
        for f in futures:
            row = f.result()
            rows.append(row)
            print(f"{row['robot']} {row['mode']} {row['name']}: {row['verdict']} "
                  f"{row.get('error_mm', '')} {row.get('message', '')}", flush=True)
            (out / 'results.yaml').write_text(yaml.safe_dump(rows, sort_keys=False))
    except KeyboardInterrupt:
        print('interrupted: stopping the running demos...', flush=True)
        STOP.set()
        with _running_lock:
            procs = list(_running)
        for p in procs:
            _stop(p)
        pool.shutdown(wait=True, cancel_futures=True)
        return 130
    pool.shutdown()
    md = table(rows)
    (out / 'results.md').write_text(md + '\n')
    print(md)
    counts = {v: sum(r['verdict'] == v for r in rows) for v in ('passed', 'failed', 'setup')}
    print(f"passed {counts['passed']}, failed {counts['failed']}, setup {counts['setup']}")
    return 1 if counts['failed'] else 0


if __name__ == '__main__':
    sys.exit(main())
