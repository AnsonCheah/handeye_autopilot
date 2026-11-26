"""Session state machine (02-features §B): seed -> bootstrap -> centring ->
planning -> execution -> solve + verify.

One plain class with no I/O. The driver (the ROS node, or a simulated robot
in tests) asks for the next action and reports its outcome:

    s = Session(...)
    while not isinstance(a := s.next_action(), Finished):
        if isinstance(a, Move):      s.on_motion(robot.move(a.joints), robot.joints())
        elif isinstance(a, Capture): s.on_capture(camera.frames(a.frames), robot.joints())

Motion outcomes: MOTION_OK; MOTION_NOT_STARTED (planning failed, the robot
did not move: a safe skip); MOTION_FAILED (execution failed or its end state
is unknown: the session stops with ``motion_failed``). Every move is checked
against the robot-agnostic limits and IK before it is sent; a recovery move
that fails ends the session (``recovery_failed``).

Internally the flow is a generator, so it reads top to bottom.
"""

from __future__ import annotations

from collections.abc import Generator
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from . import bootstrap as bs
from . import codes, handeye
from . import transforms as tf
from .boards import BoardSpec
from .camera import CameraModel
from .capture import CaptureRecord, Frame, capture
from .config import CaptureConfig, SessionConfig, SolverConfig
from .planning import IKFunction, Plan, limits, moving_points_base, plan
from .quality import Calibration, calibrate
from .solver import Solution, SolveError, solve

Mat = NDArray[np.float64]

# phases: same strings as Calibrate.Feedback PHASE_*
SEED_CHECK, BOOTSTRAP, CENTRING, PLANNING = 'seed_check', 'bootstrap', 'centring', 'planning'
MOVING, CAPTURING, SOLVING, DONE = 'moving', 'capturing', 'solving', 'done'

# motion outcomes reported by the driver
MOTION_OK, MOTION_NOT_STARTED, MOTION_FAILED = 'ok', 'not_started', 'failed'

# session failure reasons (the node maps them to Calibrate result codes)
SEED_REJECTED = 'seed_rejected'
BOARD_LOST = 'board_lost'
BOOTSTRAP_FAILED = 'bootstrap_failed'
NOT_ENOUGH_SAMPLES = 'not_enough_samples'
MOTION_FAILED_REASON = 'motion_failed'
RECOVERY_FAILED = 'recovery_failed'

LOST = {codes.NO_BOARD, codes.MARKER_SUPPORT, codes.FEW_FEATURES, codes.NEAR_BORDER,
        codes.POOR_SPREAD}


@dataclass
class Move:
    tool_pose: Mat            # base_T_tool target
    joints: Mat               # the IK solution to send as a joint goal (01-architecture §3.2)
    purpose: str


@dataclass
class Capture:
    frames: int
    purpose: str = ''


@dataclass
class SessionResult:
    calibration: Calibration | None
    records: list[CaptureRecord]
    failure: str | None = None
    message: str = ''
    plan: Plan | None = None
    bootstrap_rounds: int = 0
    plan_weak: bool = False
    skipped_moves: int = 0
    capture_purposes: list[str] = field(default_factory=list)   # per record


@dataclass
class Finished:
    result: SessionResult


Action = Move | Capture | Finished


class _Abort(Exception):
    def __init__(self, reason: str, message: str):
        super().__init__(message)
        self.reason = reason


@dataclass
class _State:
    records: list[CaptureRecord] = field(default_factory=list)
    purposes: list[str] = field(default_factory=list)
    joints: Mat | None = None
    skipped: int = 0
    seed_tool: Mat | None = None
    X: Mat | None = None                # current estimate (after bootstrap)
    Z: Mat | None = None


class Session:
    def __init__(self, mode: str, board: BoardSpec, camera: CameraModel, ik: IKFunction, *,
                 cfg: SessionConfig | None = None, capture_cfg: CaptureConfig | None = None,
                 solver_cfg: SolverConfig | None = None, seed: int = 0) -> None:
        handeye.moving(mode, np.eye(4))          # validates the mode
        self.mode, self.board, self.camera, self.ik = mode, board, camera, ik
        self.cfg = cfg or SessionConfig()
        self.capture_cfg = capture_cfg or CaptureConfig()
        self.solver_cfg = solver_cfg or SolverConfig()
        self.seed = seed
        self.phase = SEED_CHECK
        self.plan: Plan | None = None
        self._st = _State()
        self._gen = self._run()
        self._pending: Action = next(self._gen)

    # -- driver API ---------------------------------------------------------
    def next_action(self) -> Action:
        return self._pending

    def on_motion(self, outcome: bool | str, joints: Mat | None = None) -> None:
        """outcome: MOTION_OK / MOTION_NOT_STARTED / MOTION_FAILED (True/False map
        to OK / NOT_STARTED); joints: measured joints after the motion, if known."""
        if outcome is True:
            outcome = MOTION_OK
        elif outcome is False:
            outcome = MOTION_NOT_STARTED
        if outcome not in (MOTION_OK, MOTION_NOT_STARTED, MOTION_FAILED):
            raise ValueError(f'unknown motion outcome {outcome!r}')
        self._advance(Move, (outcome, joints))

    def on_capture(self, frames: list[Frame], joints: Mat | None = None) -> None:
        """frames may be empty (camera/TF timeout): recorded as a rejected capture."""
        self._advance(Capture, (list(frames), joints))

    def _advance(self, expected: type, value: object) -> None:
        if not isinstance(self._pending, expected):
            raise RuntimeError(f'expected a reply to {type(self._pending).__name__}')
        self._pending = self._gen.send(value)

    # -- helpers (sub-generators) ----------------------------------------------
    def _capture(self, purpose: str) -> Generator[Action, object, CaptureRecord]:
        reply = yield Capture(self.cfg.frames_per_capture, purpose)
        frames: list[Frame] = reply[0]  # type: ignore[index]
        joints: Mat | None = reply[1]  # type: ignore[index]
        if joints is not None:
            self._st.joints = np.asarray(joints, dtype=np.float64)
        cid = len(self._st.records)
        if not frames:
            rec = CaptureRecord(cid, None, self.camera, np.zeros(0, np.int32), None, None,
                                None, None, None, None, False, codes.NO_BOARD,
                                'no frames received (camera or TF timeout)')
        else:
            rec = capture(cid, frames, self.board, self.camera, self.capture_cfg)
        self._st.records.append(rec)
        self._st.purposes.append(purpose)
        return rec

    def _move(self, target: Mat, purpose: str, joints: Mat | None = None,
              check_limits: bool = True) -> Generator[Action, object, bool]:
        """Limits, then IK, then the motion. True only if the robot reached it.
        Raises _Abort on MOTION_FAILED (unknown robot state)."""
        st = self._st
        if check_limits and st.seed_tool is not None:
            pts = None
            if st.X is not None and st.Z is not None:
                pts = moving_points_base(self.mode, target, st.X, st.Z, self.board)
            reason, _ = limits(target, st.seed_tool, self.cfg, pts)
            if reason is not None:
                st.skipped += 1
                return False
        if joints is None:
            res = self.ik(target)
            if res.joints is None:
                st.skipped += 1
                return False
            joints = np.asarray(res.joints, dtype=float)
        reply = yield Move(target, joints, purpose)
        outcome: str = reply[0]  # type: ignore[index]
        measured: Mat | None = reply[1]  # type: ignore[index]
        if outcome == MOTION_FAILED:
            raise _Abort(MOTION_FAILED_REASON, f'motion for {purpose} failed; check the robot')
        if outcome == MOTION_NOT_STARTED:
            st.skipped += 1
            return False
        st.joints = np.asarray(measured if measured is not None else joints, dtype=np.float64)
        return True

    def _recover(self, target: Mat, purpose: str) -> Generator[Action, object, None]:
        # the target is a verified pose that already passed the limits (or the seed)
        if not (yield from self._move(target, purpose)):
            raise _Abort(RECOVERY_FAILED, f'could not return to the last verified pose '
                         f'({purpose}); stopping')

    def _finish(self, failure: str | None = None, message: str = '', **kw: object
                ) -> Finished:
        self.phase = DONE
        return Finished(SessionResult(kw.pop('calibration', None),  # type: ignore[arg-type]
                                      self._st.records, failure, message,
                                      skipped_moves=self._st.skipped,
                                      capture_purposes=list(self._st.purposes),
                                      **kw))  # type: ignore[arg-type]

    # -- the flow ---------------------------------------------------------------
    def _run(self) -> Generator[Action, object, None]:
        try:
            yield from self._flow()
        except _Abort as e:
            yield self._finish(e.reason, str(e))

    def _flow(self) -> Generator[Action, object, None]:
        cfg = self.cfg
        # 1. seed (§B1)
        seed = yield from self._capture('seed')
        if not seed.accepted:
            yield self._finish(SEED_REJECTED, f'seed view rejected ({seed.reject}): '
                               f'{seed.detail}')
            return
        assert seed.camera_T_board is not None and seed.tool_pose is not None
        tilt = seed_tilt(seed.camera_T_board, self.board)
        if tilt < cfg.min_seed_tilt:
            yield self._finish(SEED_REJECTED, f'the board is {np.degrees(tilt):.0f} deg from '
                               f'face-on; tilt it at least {np.degrees(cfg.min_seed_tilt):.0f}'
                               ' deg so its pose is unambiguous')
            return
        seed_tool, seed_C = seed.tool_pose, seed.camera_T_board
        self._st.seed_tool = seed_tool

        # 2. bootstrap with incremental excitation (§B2)
        self.phase = BOOTSTRAP
        est = yield from self._bootstrap(seed_tool, seed_C)
        if isinstance(est, Finished):
            yield est
            return
        sol, rounds = est
        X, Z = sol.X, sol.Z
        self._st.X, self._st.Z = X, Z
        uncertainty = cfg.uncertainty_margin_px + 2.0 * sol.reprojection_max_px

        # 3. bounded centring (§B3)
        self.phase = CENTRING
        yield from self._centre(X, Z, uncertainty)

        # 4. plan (§C) and execute
        self.phase = PLANNING
        captured = [r.tool_pose for r in self._st.records if r.accepted]
        p = plan(self.mode, X, Z, self.camera, self.board, seed_tool, self._st.joints,
                 captured, self.ik, cfg, self.solver_cfg, self.seed,  # type: ignore[arg-type]
                 uncertainty_px=uncertainty)
        self.plan = p           # the node reports a weak plan in feedback right away
        got = 0
        for cand in p.poses:
            if got >= p.target:
                break
            self.phase = MOVING
            if not (yield from self._move(cand.tool_pose, 'plan', cand.joints)):
                continue                       # a spare takes its place
            self.phase = CAPTURING
            rec = yield from self._capture('plan')
            got += rec.accepted

        # 5. solve, verify, gate (§E-G)
        self.phase = SOLVING
        accepted = sum(r.accepted for r in self._st.records)
        weak_msg = ('plan was weak: not enough informative poses were reachable; widen the '
                    'envelope, the orientation limit or the box') if p.weak else ''
        if accepted < self.solver_cfg.min_captures:
            yield self._finish(NOT_ENOUGH_SAMPLES, f'{accepted} accepted captures. {weak_msg}',
                               plan=p, bootstrap_rounds=rounds, plan_weak=p.weak)
            return
        cal = calibrate(self.mode, self._st.records, self.solver_cfg)
        yield self._finish(None if cal.transform_valid else
                           (cal.failure.reason if cal.failure else 'solve_failed'), weak_msg,
                           calibration=cal, plan=p, bootstrap_rounds=rounds, plan_weak=p.weak)

    def _bootstrap(self, seed_tool: Mat, seed_C: Mat
                   ) -> Generator[Action, object, tuple[Solution, int] | Finished]:
        """Rounds of +/- rotation about and translation along each tool axis, from the seed.

        Step growth: a step doubles only after a COMPLETE clean round (no skipped
        or failed move, no rejected capture, no halving) and only if the estimate
        settled since the previous round, or, before the first usable estimate
        exists, after any complete clean round. A halved step becomes the
        current step for its kind (growth resumes after a clean round). A lost
        board sends the robot back to the seed (which must succeed) and halves
        the step; after max_halvings it fails naming L_max.
        """
        cfg = self.cfg
        step = {'rotation': cfg.first_rotation_step, 'translation': cfg.first_translation_step}
        cap = {'rotation': cfg.max_rotation_step, 'translation': cfg.max_translation_step}
        boot_cfg = SolverConfig(**{**self.solver_cfg.to_dict(),
                                   'min_captures': cfg.bootstrap_min_captures,
                                   'min_rotation_amplitude': cfg.bootstrap_min_rotation,
                                   'min_pair_rotation': cfg.bootstrap_min_pair_rotation,
                                   'min_scaled_singular_value': 0.0})
        prev: Solution | None = None
        best: Solution | None = None
        rounds = 0
        margin = cfg.image_margin_px + cfg.uncertainty_margin_px
        for round_index in range(1, cfg.max_bootstrap_rounds + 1):
            rounds = round_index
            clean = True
            for kind in ('rotation', 'translation'):
                for axis in range(3):
                    for sign in (1.0, -1.0):
                        def check(v: float, kind: str = kind, axis: int = axis,
                                  sign: float = sign, est: Solution | None = best) -> bool:
                            if est is None:      # worst case over the lever arm (§B2)
                                if kind == 'rotation':
                                    return bs.rotation_is_safe(self.mode, seed_C, self.board,
                                                               self.camera, v, cfg)
                                return bs.translation_is_safe(seed_C, self.board, self.camera,
                                                              v, cfg)
                            # with a rough estimate: predicted view along the whole move
                            tgt = seed_tool @ bs.tool_step(kind, axis,
                                                           sign * v * cfg.step_safety)
                            return bs.predicted_is_safe(self.mode, est.X, est.Z, seed_tool,
                                                        tgt, self.board, self.camera, margin)
                        v = bs.safe_step(check, step[kind], step[kind] / 8)
                        if v < step[kind]:
                            clean = False          # visibility already limits this step
                        if v == 0.0:
                            continue
                        for halving in range(cfg.max_halvings + 1):
                            # every step starts at the seed, so the checked motion
                            # (seed -> seed . step) is the one the robot makes
                            yield from self._recover(seed_tool, 'bootstrap: to the seed')
                            target = seed_tool @ bs.tool_step(kind, axis, sign * v)
                            if not (yield from self._move(target, 'bootstrap')):
                                clean = False
                                break
                            rec = yield from self._capture('bootstrap')
                            if rec.accepted:
                                break
                            clean = False
                            if rec.reject not in LOST:
                                break
                            if halving == cfg.max_halvings:
                                yield from self._recover(seed_tool,
                                                         'bootstrap: back to the seed')
                                return self._finish(
                                    BOARD_LOST, f'board lost after a {kind} step of '
                                    f'{self._fmt(kind, v)} even after halving it '
                                    f'{cfg.max_halvings} times; L_max = {cfg.lever_max} m '
                                    'is probably smaller than the real lever arm between '
                                    'the tool and the ' + ('board' if self.mode ==
                                                           'eye_to_hand' else 'camera'))
                            v *= 0.5
                            # the halved step becomes the current step for its kind;
                            # growth resumes only after a complete clean round
                            step[kind] = min(step[kind], v)
            accepted = [r for r in self._st.records if r.accepted]
            try:
                sol: Solution | None = solve(self.mode, accepted, boot_cfg)
            except SolveError:
                sol = None
            settled = sol is not None and prev is not None and \
                tf.rotation_angle(sol.X, prev.X) < cfg.estimate_change_rotation and \
                tf.translation_distance(sol.X, prev.X) < cfg.estimate_change_translation
            if clean and (settled or best is None):
                for k in step:
                    step[k] = min(2 * step[k], cap[k])
            if sol is not None:
                best, prev = sol, sol
                self._st.X, self._st.Z = sol.X, sol.Z   # enables the carried-object box check
                if settled and step['rotation'] >= cap['rotation']:
                    break
        if best is None:
            return self._finish(BOOTSTRAP_FAILED, f'no usable estimate after {rounds} '
                                'bootstrap rounds; check L_max and the seed pose')
        return best, rounds

    def _centre(self, X: Mat, Z: Mat, uncertainty: float) -> Generator[Action, object, None]:
        """Move the board toward the image centre at nominal depth, from the latest
        verified view, one predicted-safe increment at a time (§B3)."""
        last = next(r for r in reversed(self._st.records) if r.accepted)
        assert last.camera_T_board is not None and last.tool_pose is not None
        good_tool, C0 = last.tool_pose, last.camera_T_board
        w, h = self.board.size_m
        centre = np.array([w / 2, h / 2, 0.0])
        start = tf.apply(C0, centre[None])[0]
        goal = np.array([0.0, 0.0, self.cfg.nominal_depth])
        R = C0[:3, :3]
        margin = self.cfg.image_margin_px + uncertainty
        for k in range(1, self.cfg.centring_steps + 1):
            p = start + (goal - start) * k / self.cfg.centring_steps
            G = handeye.tool_pose_for(self.mode, X, Z, tf.make(R, p - R @ centre))
            if not bs.predicted_is_safe(self.mode, X, Z, good_tool, G, self.board,
                                        self.camera, margin):
                return
            if not (yield from self._move(G, 'centring')):
                return
            rec = yield from self._capture('centring')
            if not rec.accepted:
                yield from self._recover(good_tool, 'centring: back to the last good view')
                return
            good_tool = rec.tool_pose if rec.tool_pose is not None else G   # measured pose

    @staticmethod
    def _fmt(kind: str, v: float) -> str:
        return f'{np.degrees(v):.1f} deg' if kind == 'rotation' else f'{v * 1000:.0f} mm'


def seed_tilt(C: Mat, board: BoardSpec) -> float:
    """Angle between the board normal and the line of sight to the board centre."""
    w, h = board.size_m
    centre = tf.apply(C, np.array([[w / 2, h / 2, 0.0]]))[0]
    cos = abs(float(C[:3, 2] @ centre)) / float(np.linalg.norm(centre))
    return float(np.arccos(np.clip(cos, -1.0, 1.0)))
