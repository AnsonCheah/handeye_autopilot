"""TF helpers (01-architecture §8-9): pose at a stamp, settle, camera subtree.

Reuse: ``tf2_ros.Buffer`` does the lookups and waiting (``lookup_transform``
with a timeout) and reports the tree (``all_frames_as_yaml``); we only convert
messages and apply the §8 settle rule and the §9 ownership rules.

ROS 2 concept: TF keeps a time-stamped tree of coordinate frames; asking for a
transform *at a stamp* returns where things were at that instant, which is how
each camera frame is paired with the robot pose it was taken at
(https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html).
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
import numpy as np
from numpy.typing import NDArray
from rclpy.duration import Duration
from rclpy.time import Time as RclTime
from scipy.spatial.transform import Rotation
import tf2_ros
import yaml

Mat = NDArray[np.float64]


def to_matrix(t: TransformStamped) -> Mat:
    q = t.transform.rotation
    p = t.transform.translation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def to_msg(T: Mat, parent: str, child: str, stamp: Time | None = None) -> TransformStamped:
    m = TransformStamped()
    if stamp is not None:
        m.header.stamp = stamp
    m.header.frame_id, m.child_frame_id = parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (
        float(v) for v in T[:3, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
     m.transform.rotation.w) = (float(v) for v in q)
    return m


def stamp_to_time(stamp: float) -> RclTime:
    sec = int(np.floor(stamp))
    return RclTime(seconds=sec, nanoseconds=int(round((stamp - sec) * 1e9)))


def lookup_at(buffer: tf2_ros.Buffer, target: str, source: str, stamp: float | int,
              timeout: float) -> Mat | None:
    """``target_T_source`` at ``stamp`` (float s, or int ns for an exact message
    stamp; waits up to ``timeout``), or None. Never substitutes the latest TF
    (§8): extrapolation or a missing stamp gives None. The wait blocks the
    calling worker thread; TF keeps arriving on the executor's other threads."""
    when = RclTime(nanoseconds=stamp) if isinstance(stamp, int) else stamp_to_time(stamp)
    try:
        t = buffer.lookup_transform(target, source, when, timeout=Duration(seconds=timeout))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException, tf2_ros.TransformException):
        return None
    return to_matrix(t)


def latest(buffer: tf2_ros.Buffer, target: str, source: str
           ) -> tuple[Mat, float] | None:
    """Latest ``target_T_source`` and its stamp (s), or None."""
    try:
        t = buffer.lookup_transform(target, source, RclTime())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException, tf2_ros.TransformException):
        return None
    return to_matrix(t), t.header.stamp.sec + t.header.stamp.nanosec * 1e-9


@dataclass
class Settled:
    ok: bool
    end_stamp: float = 0.0        # node-clock time the settle completed
    pose: Mat | None = None
    reason: str = ''


def settle(buffer: tf2_ros.Buffer, base: str, tool: str, now: callable,  # type: ignore[valid-type]
           window: float, max_translation: float, max_rotation: float, timeout: float,
           cancelled: threading.Event | None = None, poll: float = 0.02) -> Settled:
    """§8 settle: the robot's measurement is fresh (TF stamped after the settle
    start, which catches frozen joint states) AND, among those fresh samples,
    the tool pose stayed still over at least ``window`` s of stamps, with at
    least three samples in it. ``now()`` returns the node clock in s."""
    start = now()
    deadline = time.monotonic() + timeout
    fresh: list[tuple[float, Mat]] = []
    while time.monotonic() < deadline:
        if cancelled is not None and cancelled.is_set():
            return Settled(False, reason='cancelled')
        if now() < start - 0.5:
            return Settled(False, reason='the node clock jumped back during the settle')
        got = latest(buffer, base, tool)
        if got is not None:
            T, stamp = got
            if stamp > start and (not fresh or stamp > fresh[-1][0]):
                fresh.append((stamp, T))
            opens = [k for k, (st, _) in enumerate(fresh) if st <= fresh[-1][0] - window]
            if opens:                        # (float rounding can leave none yet)
                # the newest sample at or before (last - window) opens the window
                recent = fresh[max(opens):]
                ref = recent[-1][1]
                still = len(recent) >= 3 and all(
                    np.linalg.norm(Pm[:3, 3] - ref[:3, 3]) <= max_translation and
                    Rotation.from_matrix(Pm[:3, :3].T @ ref[:3, :3]).magnitude() <= max_rotation
                    for _, Pm in recent)
                if still:
                    return Settled(True, now(), ref)
        time.sleep(poll)
    if not fresh:
        return Settled(False, reason=f'no TF {base} -> {tool} newer than the settle start '
                       f'within {timeout:.1f} s (frozen joint_states, or the robot driver '
                       'stopped publishing)')
    return Settled(False, reason=f'the tool did not stay still for {window:.2f} s within '
                   f'{timeout:.1f} s')


# -- camera subtree and ownership (§9) -------------------------------------------

class StaticLinks:
    """Every (parent, child) seen on /tf_static (latched, so late joiners get all),
    or, for the dynamic instance, on /tf."""

    def __init__(self, track_values: bool = False) -> None:
        """``track_values`` only for /tf_static: a static edge has one value (a
        second, different one means two publishers disagree). /tf trackers record
        edges only: a moving link has endless values, and keeping them would make
        every message slower (TF fell behind on a moving robot in review)."""
        self.links: set[tuple[str, str]] = set()
        self.values: dict[tuple[str, str], list[Mat]] = {}   # distinct values per edge
        self._track = track_values
        self._lock = threading.Lock()

    def add(self, msg: object) -> None:
        with self._lock:
            for t in msg.transforms:  # type: ignore[attr-defined]
                edge = (t.header.frame_id, t.child_frame_id)
                self.links.add(edge)
                if not self._track:
                    continue
                T = to_matrix(t)
                seen = self.values.setdefault(edge, [])
                if len(seen) < 2 and not any(np.allclose(T, S, atol=1e-6) for S in seen):
                    seen.append(T)               # two distinct values already prove a conflict

    def conflicting(self, parent: str, child: str) -> bool:
        """Two publishers announced different values for this static edge (a
        driver and a URDF, say): listeners may end up with either."""
        with self._lock:
            return len(self.values.get((parent, child), [])) > 1

    def has(self, parent: str, child: str) -> bool:
        with self._lock:
            return (parent, child) in self.links

    def has_child(self, child: str) -> str | None:
        """A parent this child was seen under, if any (a competing driver)."""
        with self._lock:
            return next((p for p, c in self.links if c == child), None)


def parents(buffer: tf2_ros.Buffer) -> dict[str, str]:
    """child -> parent for every frame TF currently knows."""
    doc = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
    return {child: str(info['parent']) for child, info in doc.items()
            if isinstance(info, dict) and 'parent' in info}


def ancestors(tree: dict[str, str], frame: str) -> list[str]:
    """frame, its parent, ... up to the root (cycle-safe)."""
    out = [frame]
    while out[-1] in tree and tree[out[-1]] not in out:
        out.append(tree[out[-1]])
    return out


@dataclass
class Subtree:
    root: str
    chain: list[str]           # optical, ..., root
    error: str = ''


def camera_subtree(tree: dict[str, str], static: StaticLinks, optical: str, parent: str,
                   root_param: str, dynamic: StaticLinks | None = None) -> Subtree:
    """§9: find the camera driver's subtree root and check it may be attached.

    - chain optical -> ... -> root, every link published on /tf_static;
    - root has no existing parent in TF;
    - root is not an ancestor of ``parent`` (would make a cycle).
    """
    up = ancestors(tree, optical)
    if root_param:
        if root_param not in up:
            return Subtree(root_param, up, f'camera_root_frame {root_param!r} is not an '
                           f'ancestor of the optical frame {optical!r} (chain: '
                           f'{" -> ".join(up)})')
        chain = up[:up.index(root_param) + 1]
    else:
        chain = up
    root = chain[-1]
    if root in tree:
        return Subtree(root, chain, f'{root!r} already has a parent ({tree[root]!r}), from '
                       'a URDF mounting joint, a driver or a previous calibration; remove '
                       'that transform, or set camera_root_frame to a frame below it')
    if root in ancestors(tree, parent):
        return Subtree(root, chain, f'{root!r} is an ancestor of {parent!r}: publishing '
                       f'{parent} -> {root} would make a cycle; set camera_root_frame to the '
                       "camera driver's own root frame")
    for child, par in zip(chain, chain[1:], strict=False):
        moving = dynamic.has_child(child) if dynamic is not None else None
        if moving is not None:
            return Subtree(root, chain, f'{moving} -> {child} is published on /tf (moving or '
                           'a competing driver): the camera subtree must be static')
        if static.conflicting(par, child):
            return Subtree(root, chain, f'{par} -> {child} has conflicting values on '
                           '/tf_static (two publishers disagree): remove one')
        if not static.has(par, child):
            return Subtree(root, chain, f'{par} -> {child} is not on /tf_static: the camera '
                           'subtree must be static (a moving link cannot be calibrated '
                           'through)')
    return Subtree(root, chain)
