"""The moveit_msgs fields the node uses exist on this distro (01-architecture §12,
§13). Skipped without moveit_msgs locally; CI sets HANDEYE_AUTOPILOT_REQUIRE_DEPS=1
so a missing package fails instead."""

import importlib
import os

import pytest


def _msgs(name):
    try:
        return importlib.import_module(name)
    except ImportError:
        if os.environ.get('HANDEYE_AUTOPILOT_REQUIRE_DEPS') == '1':
            raise
        # shortcut: skipped locally without the apt package; CI enforces it.
        pytest.skip('moveit_msgs not installed')


def test_request_and_result_fields_we_use():
    srv = _msgs('moveit_msgs.srv')
    act = _msgs('moveit_msgs.action')
    ik = srv.GetPositionIK.Request().ik_request
    for f in ('group_name', 'ik_link_name', 'robot_state', 'avoid_collisions',
              'pose_stamped', 'timeout'):
        assert hasattr(ik, f), f
    assert hasattr(ik.robot_state, 'is_diff')
    res = srv.GetPositionIK.Response()
    assert hasattr(res.solution.joint_state, 'position') and hasattr(res.error_code, 'val')
    cart = srv.GetCartesianPath.Request()
    for f in ('header', 'start_state', 'group_name', 'link_name', 'waypoints', 'max_step',
              'avoid_collisions'):
        assert hasattr(cart, f), f
    # the jump check: one of the two spellings MoveIt has used (set where present)
    assert hasattr(cart, 'revolute_jump_threshold') or hasattr(cart, 'jump_threshold')
    assert hasattr(srv.GetCartesianPath.Response(), 'fraction')
    goal = act.MoveGroup.Goal()
    for f in ('group_name', 'num_planning_attempts', 'allowed_planning_time',
              'max_velocity_scaling_factor', 'max_acceleration_scaling_factor',
              'goal_constraints', 'path_constraints', 'start_state', 'workspace_parameters'):
        assert hasattr(goal.request, f), f
    assert hasattr(goal.planning_options, 'plan_only')
    result = act.MoveGroup.Result()
    assert hasattr(result, 'planned_trajectory') and hasattr(result, 'error_code')
    assert hasattr(act.ExecuteTrajectory.Goal(), 'trajectory')
    assert hasattr(act.ExecuteTrajectory.Result(), 'error_code')
