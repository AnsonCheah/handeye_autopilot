"""One capture: N frames -> gates -> consensus -> one capture record (02-features §D2, §D5).

The capture record is the ROS-free hand-off to the solver (M1b) and the unit
of the observations YAML.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from itertools import combinations
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import codes
from . import transforms as tf
from .boards import BoardSpec
from .camera import CameraModel
from .config import CaptureConfig
from .detection import Detection, detect
from .gates import GateOutcome, evaluate
from .pose import BoardPose

OBSERVATIONS_SCHEMA_VERSION = 1


@dataclass
class Frame:
    image: NDArray[np.uint8]                     # grayscale
    stamp: float                                 # acquisition stamp, s
    tool_pose: NDArray[np.float64] | None        # base_T_tool at `stamp`; None = no TF


@dataclass
class FrameResult:
    frame: Frame
    detection: Detection
    pose: BoardPose | None
    outcome: GateOutcome


@dataclass
class CaptureRecord:
    """§D5. Fields that did not exist when a capture was rejected are None (null)."""

    capture_id: int
    stamp: float | None
    camera: CameraModel
    ids: NDArray[np.int32]                              # [] when no board
    object_points: NDArray[np.float64] | None           # (N, 3) m
    image_points: NDArray[np.float64] | None            # (N, 2) px
    tool_pose: NDArray[np.float64] | None               # base_T_tool, 4x4
    camera_T_board: NDArray[np.float64] | None          # refined measurement, 4x4
    ippe_poses: list[NDArray[np.float64]] | None        # both raw IPPE poses, best first
    ippe_errors_px: list[float] | None                  # their RMS errors
    accepted: bool
    reject: str | None = None
    detail: str = ''
    metrics: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.accepted:
            if self.reject not in codes.ALL:
                raise ValueError(f'rejected capture {self.capture_id} has no valid code')
            return
        missing = [name for name, ok in [
            ('stamp', self.stamp is not None),
            ('ids', len(self.ids) > 0),
            ('object_points', self.object_points is not None
             and len(self.object_points) == len(self.ids)),
            ('image_points', self.image_points is not None
             and len(self.image_points) == len(self.ids)),
            ('tool_pose', self.tool_pose is not None),
            ('camera_T_board', self.camera_T_board is not None),
            ('ippe_poses', self.ippe_poses is not None and len(self.ippe_poses) == 2),
            ('ippe_errors_px', self.ippe_errors_px is not None
             and len(self.ippe_errors_px) == 2),
        ] if not ok]
        if missing:
            raise ValueError(f'accepted capture {self.capture_id} is missing solver inputs: '
                             f'{", ".join(missing)}')

    def to_dict(self) -> dict[str, Any]:
        def arr(a: Any) -> Any:
            return None if a is None else np.asarray(a).tolist()
        return {
            'capture_id': self.capture_id, 'stamp': self.stamp,
            'camera': self.camera.to_dict(), 'ids': arr(self.ids),
            'object_points': arr(self.object_points), 'image_points': arr(self.image_points),
            'tool_pose': arr(self.tool_pose), 'camera_T_board': arr(self.camera_T_board),
            'ippe_poses': None if self.ippe_poses is None else [arr(T) for T in
                                                                self.ippe_poses],
            'ippe_errors_px': None if self.ippe_errors_px is None else
            [float(e) for e in self.ippe_errors_px],
            'accepted': self.accepted, 'reject': self.reject, 'detail': self.detail,
            'metrics': {k: float(v) for k, v in self.metrics.items()},
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> CaptureRecord:
        def arr(v: Any, shape: tuple[int, ...]) -> Any:
            return None if v is None else np.asarray(v, dtype=float).reshape(shape)
        if not isinstance(d.get('accepted'), bool):
            raise ValueError(f'capture {d.get("capture_id")}: accepted must be true/false')
        poses = d.get('ippe_poses')
        if poses is not None and any(p is None for p in poses):
            raise ValueError(f'capture {d.get("capture_id")}: null IPPE pose')
        stamp = d.get('stamp')
        return cls(
            capture_id=int(d['capture_id']),
            stamp=None if stamp is None else float(stamp),
            camera=CameraModel.from_dict(d['camera']),
            ids=np.asarray(d.get('ids') or [], dtype=np.int32).reshape(-1),
            object_points=arr(d.get('object_points'), (-1, 3)),
            image_points=arr(d.get('image_points'), (-1, 2)),
            tool_pose=arr(d.get('tool_pose'), (4, 4)),
            camera_T_board=arr(d.get('camera_T_board'), (4, 4)),
            ippe_poses=None if d.get('ippe_poses') is None else
            [arr(T, (4, 4)) for T in d['ippe_poses']],
            ippe_errors_px=None if d.get('ippe_errors_px') is None else
            [float(e) for e in d['ippe_errors_px']],
            accepted=bool(d['accepted']), reject=d.get('reject'),
            detail=str(d.get('detail', '')), metrics=dict(d.get('metrics') or {}),
        )


def process_frame(frame: Frame, spec: BoardSpec, camera: CameraModel,
                  cfg: CaptureConfig) -> FrameResult:
    return process_detection(detect(frame.image, spec, camera, cfg), frame, spec, camera, cfg)


def process_detection(det: Detection, frame: Frame, spec: BoardSpec, camera: CameraModel,
                      cfg: CaptureConfig) -> FrameResult:
    outcome, pose = evaluate(det, frame.image, spec, camera, cfg)
    return FrameResult(frame, det, pose, outcome)


def capture(capture_id: int, frames: list[Frame], spec: BoardSpec, camera: CameraModel,
            cfg: CaptureConfig) -> CaptureRecord:
    """Gates on every frame, then consensus (§D2).

    Frames without TF at their stamp are dropped (at most
    ``max_frames_without_tf``); at most ``max_failed_frames`` may fail a gate;
    at least ``consensus_min_frames`` must remain, and every PAIR of them must
    agree within the limits. The kept frame is the medoid pose: a real exposure
    paired with its own TF, never an average.
    """
    if not frames:
        raise ValueError('capture() needs at least one frame')
    no_tf = [f for f in frames if f.tool_pose is None]
    results = [process_frame(f, spec, camera, cfg) for f in frames if f.tool_pose is not None]

    def reject(code: str, detail: str, src: FrameResult | None,
               first: Frame = frames[0]) -> CaptureRecord:
        return _record(capture_id, camera, src, first, accepted=False, reject=code,
                       detail=detail)

    if len(no_tf) > cfg.max_frames_without_tf:
        return reject(codes.TF_UNAVAILABLE,
                      f'{len(no_tf)} of {len(frames)} frames had no TF at their stamp', None,
                      first=no_tf[0])
    good = [r for r in results if r.outcome.accepted]
    failed = [r for r in results if not r.outcome.accepted]
    if failed and (len(failed) > cfg.max_failed_frames or
                   len(good) < cfg.consensus_min_frames):
        code, n = Counter(str(r.outcome.reject) for r in failed).most_common(1)[0]
        src = next(r for r in failed if r.outcome.reject == code)
        return reject(code, f'{len(failed)} of {len(results)} frames rejected '
                      f'({n} x {code}): {src.outcome.detail}', src)
    if len(good) < cfg.consensus_min_frames:
        return reject(codes.CONSENSUS, f'{len(good)} good frames < '
                      f'{cfg.consensus_min_frames}', good[0] if good else None)

    poses = [r.pose.best.camera_T_board for r in good if r.pose is not None]
    k = medoid(poses)
    spread_t, spread_r = pairwise_spread(poses)
    dist = float(np.linalg.norm(poses[k][:3, 3]))
    if spread_t > cfg.consensus_max_translation_frac * dist or \
            spread_r > cfg.consensus_max_rotation:
        return reject(codes.CONSENSUS,
                      f'frames disagree by up to {spread_t * 1000:.2f} mm, '
                      f'{np.degrees(spread_r):.2f} deg', good[k])
    rec = _record(capture_id, camera, good[k], frames[0], accepted=True)
    rec.metrics.update(consensus_translation_spread=spread_t,
                       consensus_rotation_spread=spread_r, frames_used=float(len(good)))
    return rec


def medoid(poses: list[NDArray[np.float64]]) -> int:
    """Index of the pose with the smallest summed (normalised) distance to the others."""
    if len(poses) == 1:
        return 0
    dt = np.array([[tf.translation_distance(a, b) for b in poses] for a in poses])
    dr = np.array([[tf.rotation_angle(a, b) for b in poses] for a in poses])
    st = max(float(np.median(dt[dt > 0])) if (dt > 0).any() else 1.0, 1e-12)
    sr = max(float(np.median(dr[dr > 0])) if (dr > 0).any() else 1.0, 1e-12)
    return int(np.argmin((dt / st + dr / sr).sum(axis=1)))


def pairwise_spread(poses: list[NDArray[np.float64]]) -> tuple[float, float]:
    """Largest translation and rotation difference over all pairs of poses."""
    pairs = list(combinations(poses, 2))
    if not pairs:
        return 0.0, 0.0
    return (max(tf.translation_distance(a, b) for a, b in pairs),
            max(tf.rotation_angle(a, b) for a, b in pairs))


def _record(capture_id: int, camera: CameraModel, r: FrameResult | None, first: Frame, *,
            accepted: bool, reject: str | None = None, detail: str = '') -> CaptureRecord:
    if r is None:
        return CaptureRecord(capture_id, first.stamp, camera, np.zeros(0, np.int32), None,
                             None, None, None, None, None, accepted, reject, detail)
    det, pose = r.detection, r.pose
    has_points = det.ok and len(det.ids) > 0
    return CaptureRecord(
        capture_id, r.frame.stamp, camera, det.ids,
        det.object_points if has_points else None,
        det.image_points if has_points else None,
        r.frame.tool_pose,
        None if pose is None else pose.best.camera_T_board,
        None if pose is None else [h.camera_T_board for h in pose.hypotheses],
        None if pose is None else [h.rms_px for h in pose.hypotheses],
        accepted, None if accepted else reject, '' if accepted else detail,
        dict(r.outcome.metrics))
