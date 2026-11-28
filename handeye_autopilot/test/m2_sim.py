"""Simulated robot + camera for M2 session tests (spec/m2.yaml).

Perfect motion (the robot reaches the commanded pose exactly), rendered
images of the true board pose, noisy reported tool poses, and a fake IK that
accepts poses inside a reach shell. The board pose comes from the physical
chain (m1b_sim.tool_from_board's inverse), not from the mode adapter.
"""

from __future__ import annotations

import contextlib
import faulthandler
import math
from pathlib import Path
import signal
import time

from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.boards import board_texture
from handeye_autopilot.core.capture import Frame
from handeye_autopilot.core.config import CaptureConfig, SessionConfig, SolverConfig
from handeye_autopilot.core.planning import IKResult
from handeye_autopilot.core.render import ImageEffects, PlanarObject, Renderer, look_at_board
from handeye_autopilot.core.session import Capture, Finished, Move, Session
from m1b_sim import BOARD, CAMERA, robot_noise, tool_from_board, truth
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

SPEC = yaml.safe_load((Path(__file__).parent / 'spec' / 'm2.yaml').read_text())


def board_in_camera(mode, X, Z, G):
    """Physical chain: where the board really is for tool pose G."""
    if mode == 'eye_to_hand':
        return tf.inv(X) @ G @ Z
    return tf.inv(X) @ tf.inv(G) @ Z


def fake_fk(joints):
    """Inverse of FakeIK's joint encoding: the simulated robot moves by its joints."""
    j = np.asarray(joints, dtype=float)
    return tf.make(Rotation.from_rotvec(j[3:6]), j[:3])


class FakeIK:
    """Reachable inside a shell around the base; joints = position + rotation vector."""

    def __init__(self, r_min, r_max, calls=None):
        self.r_min, self.r_max, self.calls = r_min, r_max, 0

    def __call__(self, G):
        self.calls += 1
        r = float(np.linalg.norm(G[:3, 3]))
        if not self.r_min <= r <= self.r_max:
            return IKResult(None, 'NO_IK_SOLUTION')
        return IKResult(np.r_[G[:3, 3], Rotation.from_matrix(G[:3, :3]).as_rotvec()])


def seed_pose(mode, X, Z, s=None):
    s = s or SPEC['seed']
    w, h = BOARD.size_m
    C = look_at_board(np.array([w / 2, h / 2, 0]), s['distance'], math.radians(s['tilt'][0]),
                      math.radians(s['tilt'][1]), math.radians(s['roll']),
                      offset_px=tuple(s['offset']), camera=CAMERA)
    return tool_from_board(mode, X, Z, C)


def run(mode, *, seed=0, session_cfg=None, ik=None, true_lever_scale=1.0, seed_spec=None,
        frames=1, lose=(), fail_move=None, log=None, mounts=None):
    """One full session. ``lose``: capture indices rendered without the board
    (board lost); ``fail_move(index, purpose)``: True makes that motion fail;
    ``log``: list that receives every action. Motion goes through fake FK from the
    Move's joints (so a wrong joint goal moves the robot to the wrong place).
    ``fail_move`` may return True (not started) or a session.MOTION_* outcome.
    ``mounts``: (X, Z) instead of the spec's truth (the mount sweep)."""
    X, Z = mounts if mounts is not None else truth(mode)
    if true_lever_scale != 1.0:          # make the real lever arm longer than assumed
        if mode == 'eye_to_hand':
            Z = tf.make(Z[:3, :3], Z[:3, 3] * true_lever_scale)
        else:
            X = tf.make(X[:3, :3], X[:3, 3] * true_lever_scale)
    rng = np.random.default_rng(seed)
    tex, H = board_texture(BOARD, 60, 30)
    renderer = Renderer(CAMERA, ImageEffects(noise_sigma=2.0))
    noise = robot_noise()
    cfg = session_cfg or SessionConfig(**{**SessionConfig().to_dict(), **SPEC['session'],
                                          'frames_per_capture': frames})
    ik = ik or FakeIK(*SPEC['reach'])
    s = Session(mode, BOARD, CAMERA, ik, cfg=cfg,
                capture_cfg=CaptureConfig(consensus_min_frames=frames, max_failed_frames=0),
                solver_cfg=SolverConfig(), seed=seed)
    G = seed_pose(mode, X, Z, seed_spec)
    t0, stamp = time.time(), 0.0
    moves = captures = 0
    with _deadline(SPEC['deadline_s']):
        return _drive(s, mode, X, Z, G, t0, stamp, moves, captures, rng, tex, H, renderer,
                      noise, ik, frames, lose, fail_move, log)


@contextlib.contextmanager
def _deadline(seconds):
    """Fail a hung session at the deadline, even inside a call (SIGALRM, Linux)."""
    def boom(signum, frame):
        raise TimeoutError(f'session exceeded the {seconds} s test deadline')
    old = signal.signal(signal.SIGALRM, boom)
    signal.alarm(int(seconds))
    # backstop: a native call can hold off the Python handler; kill the
    # process (with tracebacks) if it is still stuck well past the deadline
    # (the alarm itself fires as soon as a native call returns; this catches a
    # call that never returns)
    faulthandler.dump_traceback_later(seconds + 5, exit=True)
    try:
        yield
    finally:
        faulthandler.cancel_dump_traceback_later()
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)


def _drive(s, mode, X, Z, G, t0, stamp, moves, captures, rng, tex, H, renderer, noise, ik,
           frames, lose, fail_move, log):
    while True:
        a = s.next_action()
        if isinstance(a, Finished):
            r = a.result
            r.elapsed = time.time() - t0          # type: ignore[attr-defined]
            r.moves, r.captures = moves, captures  # type: ignore[attr-defined]
            return r, X, Z
        if log is not None:
            log.append(a)
        if isinstance(a, Move):
            moves += 1
            outcome = fail_move(moves, a.purpose) if fail_move is not None else None
            if outcome:
                s.on_motion('not_started' if outcome is True else outcome)
                continue
            G = fake_fk(a.joints)                  # perfect motion, through the joints
            s.on_motion('ok', a.joints, G)         # measured joints and TF tool pose
        elif isinstance(a, Capture):
            captures += 1
            C = board_in_camera(mode, X, Z, G)
            objects = [] if (captures - 1) in lose else [PlanarObject(tex, H, C)]
            fs = []
            for _ in range(a.frames):
                stamp += 0.033
                fs.append(Frame(renderer.render(objects, rng), stamp, noise.report(G, rng)))
            s.on_capture(fs, ik(G).joints)
