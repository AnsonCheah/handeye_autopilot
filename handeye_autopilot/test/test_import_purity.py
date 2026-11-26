"""The core and the top-level package must not import ROS (01-architecture §4)."""

import ast
import pathlib
import subprocess
import sys

PKG = pathlib.Path(__file__).resolve().parents[1] / 'handeye_autopilot'

# Top-level module names that mean "this needs ROS".
ROS_MODULES = {
    'rclpy', 'rosidl_runtime_py', 'launch', 'launch_ros', 'tf2_ros', 'tf2_py',
    'cv_bridge', 'image_geometry', 'pymoveit2', 'message_filters',
    'ament_index_python', 'builtin_interfaces',
}
ROS_SUFFIXES = ('_msgs', '_srvs', '_interfaces')


def _is_ros(name: str) -> bool:
    top = name.split('.')[0]
    return top in ROS_MODULES or top.endswith(ROS_SUFFIXES)


def _imports(path: pathlib.Path):
    tree = ast.parse(path.read_text(), filename=str(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            yield node.module


def _checked_files():
    yield PKG / '__init__.py'
    yield from sorted((PKG / 'core').rglob('*.py'))


def test_detector_flags_ros_names():
    assert _is_ros('rclpy.node')
    assert _is_ros('geometry_msgs.msg')
    assert _is_ros('handeye_autopilot_interfaces.action')
    assert not _is_ros('numpy')
    assert not _is_ros('cv2')


def test_core_has_no_ros_imports_static():
    offenders = [
        f'{path.relative_to(PKG.parent)}: {name}'
        for path in _checked_files()
        for name in _imports(path)
        if _is_ros(name)
    ]
    assert not offenders, 'ROS imports in core:\n' + '\n'.join(offenders)


def test_core_imports_without_ros_at_runtime():
    """Import every core module in a fresh interpreter; no ROS module may load."""
    modules = ['handeye_autopilot'] + [
        'handeye_autopilot.' + '.'.join(p.relative_to(PKG).with_suffix('').parts)
        .removesuffix('.__init__')
        for p in sorted((PKG / 'core').rglob('*.py'))
    ]
    code = (
        'import importlib, sys\n'
        f'for m in {modules!r}: importlib.import_module(m)\n'
        'print("\\n".join(sorted(sys.modules)))\n'
    )
    out = subprocess.run(
        [sys.executable, '-c', code], check=True, capture_output=True, text=True,
        cwd=PKG.parent,
    ).stdout.split()
    loaded = [m for m in out if _is_ros(m)]
    assert not loaded, f'ROS modules loaded by core: {loaded}'
