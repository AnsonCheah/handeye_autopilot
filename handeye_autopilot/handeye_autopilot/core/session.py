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

from collections import Counter
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
from .planning import IKFunction, Plan, lever_ball_points, limits, plan
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
    # True: a short, checked move (bootstrap, centring, recovery) whose path
    # must be the straight tool motion the check assumed: the driver plans a
    # Cartesian path through ``waypoints`` (tool poses, converted to the IK link,
    # so an offset TCP still follows the checked path) and reports
    # MOTION_NOT_STARTED if MoveIt cannot plan all of it.
    cartesian: bool = False
    waypoints: list[Mat] = field(default_factory=list)


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
    skip_reasons: Counter[str] = field(default_factory=Counter)  # why moves were skipped


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
    skip_reasons: Counter[str] = field(default_factory=Counter)
    at_verified: bool = False           # robot is at the pose of the last accepted capture
    at_pose: Mat | None = None          # newest measured (or commanded) tool pose
    frame_poses: list[Mat] = field(default_factory=list)   # last capture's measured poses
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

    def on_motion(self, outcome: str, joints: Mat | None = None,
                  tool_pose: Mat | None = None) -> None:
        """outcome: MOTION_OK / MOTION_NOT_STARTED / MOTION_FAILED, explicitly (a
        bare False must never become a safe skip: it may hide a partial motion);
        joints / tool_pose: measured joints and TF tool pose after the motion, if
        known. The next move is checked from the measured pose."""
        if not isinstance(outcome, str) or \
                outcome not in (MOTION_OK, MOTION_NOT_STARTED, MOTION_FAILED):
            raise ValueError(f'unknown motion outcome {outcome!r}')
        self._advance(Move, (outcome, joints, tool_pose))

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
        self._st.at_verified = bool(rec.accepted)
        # Every frame's TF tool pose is a measurement of where the robot is: all
        # must be inside the limits, and the newest one is where the next move
        # starts (not only the frame consensus selected).
        measured = sorted((f for f in frames if f.tool_pose is not None), key=lambda f: f.stamp)
        self._st.frame_poses = [np.asarray(f.tool_pose, dtype=float) for f in measured]
        if measured:
            self._st.at_pose = self._st.frame_poses[-1]
            if self._st.seed_tool is not None:
                for f in measured:
                    reason = self._limit_reason(np.asarray(f.tool_pose, dtype=float))
                    if reason is not None:
                        raise _Abort(MOTION_FAILED_REASON, 'a measured tool pose at capture is '
                                     f'outside the limits ({reason}); check the robot')
        return rec

    def _move(self, target: Mat, purpose: str, joints: Mat | None = None,
              check_limits: bool = True, cartesian: bool = True
              ) -> Generator[Action, object, bool]:
        """Limits, then IK, then the motion. True only if the robot reached it.
        Raises _Abort on MOTION_FAILED (unknown robot state)."""
        st = self._st
        start = st.at_pose if st.at_pose is not None else target
        waypoints = tf.path(start, target, self.cfg.waypoint_max_angle,
                            self.cfg.waypoint_max_step) if cartesian else [target]
        if check_limits and st.seed_tool is not None:
            # a checked move is limit-checked along its whole straight path (the
            # carried object can cross a box face between two good end poses);
            # a planned joint-goal move only at its end (MoveIt guards its path)
            # segment angle between checked waypoints; 0 for a joint-goal move,
            # whose path is MoveIt's (only its end pose is checked here)
            sag_angle = max((tf.rotation_angle(a, b) for a, b in
                             zip([start] + waypoints[:-1], waypoints, strict=True)),
                            default=0.0) if cartesian else 0.0
            # the start too: the first segment bulges out from it as much as any
            checked = ([start] if cartesian and start is not target else []) + waypoints
            for G in checked:
                reason = self._limit_reason(G, sag_angle)
                if reason is not None:
                    return self._skip(reason)
        if joints is None:
            res = self.ik(target)
            if res.joints is None:
                return self._skip(f'ik_failed: {res.reason or "unknown"}')
            joints = np.asarray(res.joints, dtype=float)
        reply = yield Move(target, joints, purpose, cartesian,
                           waypoints if cartesian else [])
        outcome: str = reply[0]  # type: ignore[index]
        measured: Mat | None = reply[1]  # type: ignore[index]
        measured_pose: Mat | None = reply[2]  # type: ignore[index]
        if outcome == MOTION_FAILED:
            raise _Abort(MOTION_FAILED_REASON, f'motion for {purpose} failed; check the robot')
        if outcome == MOTION_NOT_STARTED:
            if measured is not None:        # fresh joints still order the next spare
                st.joints = np.asarray(measured, dtype=np.float64)
            if measured_pose is not None:   # and the measured pose is where it really is
                st.at_pose = np.asarray(measured_pose, dtype=float)
                if check_limits and st.seed_tool is not None and \
                        self._limit_reason(st.at_pose) is not None:
                    raise _Abort(MOTION_FAILED_REASON, 'the measured tool pose is outside '
                                 f'the limits ({self._limit_reason(st.at_pose)}); '
                                 'check the robot')
            return self._skip('not_started')
        st.at_verified = False
        st.at_pose = target if measured_pose is None else np.asarray(measured_pose, dtype=float)
        if check_limits and st.seed_tool is not None and measured_pose is not None and \
                self._limit_reason(st.at_pose) is not None:
            raise _Abort(MOTION_FAILED_REASON, f'after the {purpose} move the measured tool '
                         f'pose is outside the limits ({self._limit_reason(st.at_pose)}); '
                         'check the robot')
        st.joints = np.asarray(measured if measured is not None else joints, dtype=np.float64)
        return True

    def _skip(self, reason: str) -> bool:
        self._st.skipped += 1
        self._st.skip_reasons[reason] += 1
        return False

    def _limit_reason(self, G: Mat, sag_angle: float = 0.0) -> str | None:
        """Limits at tool pose G.

        Carried object (optional box check): always the lever-bound ball, never
        the estimate (a rough estimate can misplace a 100 mm camera offset by
        40 mm; L_max is the user's guarantee). The ball is centred on the tool
        origin, so it is unchanged by rotation and sweeps a convex hull along
        the straight tool path: checking the waypoints is exact for the box.
        Orientation from the seed can exceed the endpoints' by half a segment."""
        st = self._st
        assert st.seed_tool is not None
        pts = lever_ball_points(self.mode, G, self.cfg, self.board)
        return limits(G, st.seed_tool, self.cfg, pts, 0.0, sag_angle / 2.0)[0]

    def _recover(self, target: Mat, purpose: str) -> Generator[Action, object, None]:
        # the target is a verified pose that already passed the limits (or the seed)
        at = self._st.at_pose
        if at is not None and tf.translation_distance(at, target) < 1e-9 and \
                tf.rotation_angle(at, target) < 1e-9:
            return                     # already there (the last move was skipped)
        before = Counter(self._st.skip_reasons)
        if not (yield from self._move(target, purpose)):
            why = ', '.join((self._st.skip_reasons - before).keys()) or 'unknown'
            raise _Abort(RECOVERY_FAILED, f'could not return to the last verified pose '
                         f'({purpose}: {why}); stopping')

    def _finish(self, failure: str | None = None, message: str = '', **kw: object
                ) -> Finished:
        self.phase = DONE
        return Finished(SessionResult(kw.pop('calibration', None),  # type: ignore[arg-type]
                                      self._st.records, failure, message,
                                      skipped_moves=self._st.skipped,
                                      skip_reasons=Counter(self._st.skip_reasons),
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
        # the seed and every measured pose of the seed capture (the frames were
        # taken before the seed was known, so _capture could not check them)
        if any(self._limit_reason(G) is not None
               for G in [seed_tool] + self._st.frame_poses):
            yield self._finish(SEED_REJECTED, 'the seed pose is outside the safety box'
                               + (' (with the carried object within L_max)'
                                  if cfg.box_check_moving_object else '')
                               + '; jog inside it or move/enlarge the box')
            return
        if self._st.at_pose is None:       # no TF in the frames (cannot happen when
            self._st.at_pose = seed_tool   # accepted, kept for safety)

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
        self.plan = p           # the node reports a weak plan (p.advice) in feedback now,
        # before execution; the plan still runs and the final gates decide (§C6).
        # shortcut: a spare replacing a skipped pose is not re-scored for
        # information; the solver's observability gate on the real captures
        # decides. Re-plan from the captured set if this ever leaves runs weak.
        got = 0
        queue, spares = list(p.poses[:p.target]), list(p.poses[p.target:])
        while got < p.target and (queue or spares):
            if queue:
                cand = queue.pop(0)
            else:              # a spare: the one nearest the robot's current joints
                cur = self._st.joints
                cand = spares[0] if cur is None else min(
                    spares, key=lambda c: float(np.linalg.norm(c.joints - cur))
                    if c.joints is not None else np.inf)
                spares.remove(cand)
            self.phase = MOVING
            if not (yield from self._move(cand.tool_pose, 'plan', cand.joints, cartesian=False)):
                continue                       # a spare takes its place
            self.phase = CAPTURING
            rec = yield from self._capture('plan')
            got += rec.accepted

        # 5. solve, verify, gate (§E-G)
        self.phase = SOLVING
        accepted = sum(r.accepted for r in self._st.records)
        weak_msg = p.advice
        if got < p.target:          # spares ran out: the plan's information is not met
            weak_msg = (weak_msg + ' ' if weak_msg else '') + (
                f'only {got} of the {p.target} planned captures succeeded (spares ran out)')
        if accepted < self.solver_cfg.min_captures:
            yield self._finish(NOT_ENOUGH_SAMPLES, f'{accepted} accepted captures. {weak_msg}',
                               plan=p, bootstrap_rounds=rounds,
                               plan_weak=p.weak or got < p.target)
            return
        cal = calibrate(self.mode, self._st.records, self.solver_cfg)
        yield self._finish(None if cal.transform_valid else
                           (cal.failure.reason if cal.failure else 'solve_failed'), weak_msg,
                           calibration=cal, plan=p, bootstrap_rounds=rounds,
                           plan_weak=p.weak or got < p.target)

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
                            return bs.predicted_is_safe(
                                self.mode, est.X, est.Z, seed_tool, tgt, self.board,
                                self.camera, margin + 2.0 * est.reprojection_max_px)
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
            settled = sol is not None and prev is not None and settled_estimate(sol, prev, cfg)
            if clean and (settled or best is None):
                for k in step:
                    step[k] = min(2 * step[k], cap[k])
            if sol is not None:
                best, prev = sol, sol
                self._st.X, self._st.Z = sol.X, sol.Z   # enables the carried-object box check
                if settled and step['rotation'] >= cap['rotation']:
                    break
        if best is None:
            why = ', '.join(f'{k} x{n}' for k, n in self._st.skip_reasons.most_common(3))
            return self._finish(BOOTSTRAP_FAILED, f'no usable estimate after {rounds} '
                                'bootstrap rounds; check L_max and the seed pose'
                                + (f' (skipped moves: {why})' if why else ''))
        return best, rounds

    def _centre(self, X: Mat, Z: Mat, uncertainty: float) -> Generator[Action, object, None]:
        """Move the board toward the image centre at nominal depth, from the latest
        verified view, one predicted-safe increment at a time (§B3)."""
        if not self._st.at_verified:
            # the robot ended bootstrap at a pose whose capture was rejected (or
            # unknown): go back to the seed (the reverse of a checked step) and
            # centre from the seed view, never from an older view it is not at
            assert self._st.seed_tool is not None
            yield from self._recover(self._st.seed_tool, 'centring: back to the seed')
            last = next(r for r, why in zip(self._st.records, self._st.purposes, strict=True)
                        if why == 'seed')
        else:
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
            # from where the robot is measured to be (the move starts there)
            here = self._st.at_pose if self._st.at_pose is not None else good_tool
            if not bs.predicted_is_safe(self.mode, X, Z, here, G, self.board,
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


def settled_estimate(sol: Solution, prev: Solution, cfg: SessionConfig) -> bool:
    """Both X and Z (the prediction uses both) moved less than the thresholds."""
    return all(tf.rotation_angle(a, b) < cfg.estimate_change_rotation and
               tf.translation_distance(a, b) < cfg.estimate_change_translation
               for a, b in ((sol.X, prev.X), (sol.Z, prev.Z)))
