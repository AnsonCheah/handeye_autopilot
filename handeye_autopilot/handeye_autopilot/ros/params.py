"""Node parameters and profiles (01-architecture §5a).

One ROS parameter per field of the core config dataclasses (``capture.*``,
``solver.*``, ``session.*``, ``board.*``), generated from the dataclasses so the
two cannot drift, plus frames, timing and output settings. A profile is a YAML
file ``<profiles_dir>/<name>.yaml`` with the same groups; it overrides the node
parameters for one run. Everything is snapshotted into a ``RunConfig`` when a
goal is accepted.

ROS 2 concept: parameters are typed key/value settings owned by a node, set
from YAML files or the command line and readable at run time
(https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html).
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import math
from pathlib import Path
from typing import Any

from rclpy.node import Node
import yaml

from ..core.boards import BoardSpec
from ..core.config import CaptureConfig, SessionConfig, SolverConfig

GROUPS = {'capture': CaptureConfig, 'solver': SolverConfig, 'session': SessionConfig}

BOARD_DEFAULTS: dict[str, Any] = {
    'kind': 'charuco', 'squares_x': 7, 'squares_y': 5, 'square_length': 0.04,
    'marker_length': 0.03, 'dictionary': 'DICT_5X5_100',
}


@dataclass(frozen=True)
class Timing:
    settle_window: float = 0.5          # s the tool pose must be still before frames count
    settle_translation: float = 0.0005  # m, max motion within the window
    settle_rotation: float = math.radians(0.05)
    settle_timeout: float = 10.0        # s to reach a settled, fresh pose
    tf_timeout: float = 0.5             # s to wait for TF at a frame's stamp
    clock_tolerance: float = 0.5        # s, |image stamp - node clock| allowed
    camera_info_timeout: float = 5.0    # s for CameraInfo + first image at preflight
    frame_timeout: float = 5.0          # s to collect the frames of one capture

    def __post_init__(self) -> None:
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or \
                    not math.isfinite(v) or v <= 0:
                raise ValueError(f'timing.{f.name} must be a finite number > 0')


FRAME_DEFAULTS = {'base_frame': 'base_link', 'tool_frame': 'tool0',
                  'board_frame': 'calibration_board', 'camera_root_frame': ''}


@dataclass(frozen=True)
class RunConfig:
    """Everything one run uses, fixed at goal acceptance."""

    board: BoardSpec
    capture: CaptureConfig
    solver: SolverConfig
    session: SessionConfig
    timing: Timing
    frames: dict[str, str]
    image_is_rectified: bool = False
    output_dir: str = ''
    profile: str = ''
    snapshot: dict[str, Any] = field(default_factory=dict)   # for the result YAML


def _ros_value(v: Any) -> Any:
    """Dataclass default -> a value rclpy can declare (tuples become lists)."""
    if isinstance(v, tuple):
        return [float(x) for x in v]
    return v


def declare(node: Node) -> None:
    """Declare every parameter with its default (config/default.yaml may override)."""
    for group, cls in GROUPS.items():
        for f in fields(cls):
            node.declare_parameter(f'{group}.{f.name}', _ros_value(getattr(cls(), f.name)))
    for k, v in BOARD_DEFAULTS.items():
        node.declare_parameter(f'board.{k}', v)
    for f in fields(Timing):
        node.declare_parameter(f'timing.{f.name}', getattr(Timing(), f.name))
    for k, v in FRAME_DEFAULTS.items():
        node.declare_parameter(k, v)
    node.declare_parameter('image_is_rectified', False)
    node.declare_parameter('output_dir', str(Path.home() / '.ros' / 'handeye_autopilot'))
    node.declare_parameter('profiles_dir', '')
    node.declare_parameter('testing.allow_fault_injection', False)


def _group_values(node: Node, prefix: str, names: list[str]) -> dict[str, Any]:
    out = {}
    for n in names:
        v = node.get_parameter(f'{prefix}.{n}').value
        out[n] = tuple(v) if isinstance(v, (list, tuple)) else v
    return out


def node_values(node: Node) -> dict[str, Any]:
    """The node's current parameters as a nested mapping (profile file layout)."""
    doc: dict[str, Any] = {}
    for group, cls in GROUPS.items():
        doc[group] = _group_values(node, group, [f.name for f in fields(cls)])
    doc['board'] = _group_values(node, 'board', list(BOARD_DEFAULTS))
    doc['timing'] = _group_values(node, 'timing', [f.name for f in fields(Timing)])
    for k in FRAME_DEFAULTS:
        doc[k] = node.get_parameter(k).value
    doc['image_is_rectified'] = node.get_parameter('image_is_rectified').value
    doc['output_dir'] = node.get_parameter('output_dir').value
    return doc


def profile_path(node: Node, profile: str) -> Path | None:
    """The file for a profile name, or None when it does not exist (admission)."""
    if not profile:
        return None
    if '/' in profile or profile.startswith('.'):
        return None
    d = node.get_parameter('profiles_dir').value
    if not d:
        return None
    p = Path(d).expanduser() / f'{profile}.yaml'
    return p if p.is_file() else None


def profile_exists(node: Node, profile: str) -> bool:
    return not profile or profile_path(node, profile) is not None


def merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Group-wise override; unknown groups or keys are errors (typos must not pass)."""
    out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.items()}
    for k, v in override.items():
        if k not in out:
            raise ValueError(f'unknown profile key {k!r}')
        if isinstance(out[k], dict):
            if not isinstance(v, dict):
                raise ValueError(f'profile key {k!r} must be a mapping')
            unknown = set(v) - set(out[k])
            if unknown:
                raise ValueError(f'unknown {k} keys in profile: {sorted(unknown)}')
            out[k].update({kk: tuple(vv) if isinstance(vv, list) else vv
                           for kk, vv in v.items()})
        else:
            out[k] = v
    return out


def check_types(group: str, cls: type, values: dict[str, Any]) -> None:
    """Each value has its field's type: a quoted "false" must not become True,
    a bool must not pass as a number (YAML profiles are hand-written)."""
    defaults = {f.name: getattr(cls(), f.name) for f in fields(cls)}
    for k, v in values.items():
        d = defaults.get(k)
        if isinstance(d, bool):
            ok = isinstance(v, bool)
        elif isinstance(d, int):
            ok = isinstance(v, int) and not isinstance(v, bool)
        elif isinstance(d, float):
            ok = isinstance(v, (int, float)) and not isinstance(v, bool)
        elif isinstance(d, tuple):
            ok = isinstance(v, (tuple, list)) and all(
                isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)
        else:
            ok = True
        if not ok:
            raise ValueError(f'{group}.{k} must be a {type(d).__name__}, got {v!r}')


def build(doc: dict[str, Any], profile: str = '') -> RunConfig:
    """Validated RunConfig from a nested mapping; raises ValueError with the reason."""
    for k in ('base_frame', 'tool_frame', 'board_frame'):
        if not isinstance(doc.get(k), str) or not doc[k]:
            raise ValueError(f'{k} must be a non-empty frame name')
    if not isinstance(doc.get('camera_root_frame', ''), str):
        raise ValueError('camera_root_frame must be a string')
    for k in ('image_is_rectified',):
        if not isinstance(doc.get(k), bool):          # "false" (a string) must not pass
            raise ValueError(f'{k} must be true or false, got {doc.get(k)!r}')
    if not isinstance(doc.get('output_dir'), str):
        raise ValueError('output_dir must be a string')
    for group in ('board', 'capture', 'solver', 'session', 'timing'):
        if not isinstance(doc.get(group), dict):
            raise ValueError(f'{group} must be a mapping')
    if doc['base_frame'] == doc['tool_frame']:
        raise ValueError('base_frame and tool_frame must differ')
    for group, cls in (('capture', CaptureConfig), ('solver', SolverConfig),
                       ('session', SessionConfig), ('timing', Timing)):
        check_types(group, cls, doc[group])
    board = BoardSpec.from_dict(dict(doc['board']))
    return RunConfig(
        board=board,
        capture=CaptureConfig.from_dict(dict(doc['capture'])),
        solver=SolverConfig.from_dict(dict(doc['solver'])),
        session=SessionConfig(**dict(doc['session'])),
        timing=Timing(**dict(doc['timing'])),
        frames={k: str(doc[k]) for k in FRAME_DEFAULTS},
        image_is_rectified=doc['image_is_rectified'],
        output_dir=doc['output_dir'],
        profile=profile,
        snapshot=doc,
    )


def load_run_config(node: Node, profile: str, snapshot: dict[str, Any] | None = None,
                    profile_text: str | None = None) -> RunConfig:
    """Node parameters (``snapshot``: as taken at goal acceptance), overridden by
    the profile (``profile_text``: the file as read at acceptance)."""
    doc = snapshot if snapshot is not None else node_values(node)
    if profile:
        if profile_text is None:
            path = profile_path(node, profile)
            if path is None:
                raise ValueError(f'profile {profile!r} not found')
            profile_text = path.read_text()
        try:
            over = yaml.safe_load(profile_text)
        except yaml.YAMLError as e:
            raise ValueError(f'profile {profile!r}: {e}') from e
        if over is None:
            over = {}
        if not isinstance(over, dict):
            raise ValueError(f'profile {profile!r} must be a mapping')
        doc = merge(doc, over)
    try:
        return build(doc, profile)
    except (TypeError, KeyError) as e:
        raise ValueError(f'invalid parameters: {e}') from e
