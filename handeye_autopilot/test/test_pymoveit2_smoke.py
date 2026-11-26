"""Smoke-test the pymoveit2 API we depend on (03-milestones M0 dependency gate).

Locally the test is skipped when pymoveit2 isn't installed; CI sets
HANDEYE_AUTOPILOT_REQUIRE_DEPS=1 so a missing package fails instead.
"""

import importlib
import inspect
import os

import pytest


def _pymoveit2():
    try:
        return importlib.import_module('pymoveit2')
    except ImportError:
        if os.environ.get('HANDEYE_AUTOPILOT_REQUIRE_DEPS') == '1':
            raise
        # shortcut: skipped locally without the apt package; CI enforces it.
        pytest.skip('pymoveit2 not installed (sudo apt install ros-$ROS_DISTRO-pymoveit2)')


def test_moveit2_api_we_use():
    cls = _pymoveit2().MoveIt2
    for name in ('move_to_configuration', 'cancel_execution', 'wait_until_executed'):
        assert callable(getattr(cls, name, None)), f'MoveIt2.{name} missing'
    params = inspect.signature(cls.__init__).parameters
    for name in ('node', 'joint_names', 'base_link_name', 'end_effector_name', 'group_name',
                 'use_move_group_action', 'callback_group'):
        assert name in params, f'MoveIt2.__init__ has no {name!r}'
    for attr in ('max_velocity', 'max_acceleration', 'allowed_planning_time'):
        assert isinstance(getattr(cls, attr, None), property), f'MoveIt2.{attr} missing'
