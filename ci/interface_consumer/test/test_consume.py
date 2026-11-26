"""A downstream package can import, construct and serialize the interfaces."""

import math

from rclpy.serialization import deserialize_message, serialize_message

from handeye_autopilot_interfaces.action import Calibrate
from handeye_autopilot_interfaces.msg import GateResult, QualityReport


def test_result_with_gates_round_trips():
    gates = [
        GateResult(name='min_inliers', group=GateResult.GROUP_SANITY, unit='count',
                   comparison='>=', value=18.0, limit=12.0, gated=True, passed=True),
        GateResult(name='heldout_reprojection_rms', group=GateResult.GROUP_ACCURACY_EVIDENCE,
                   unit='px', comparison='<=', value=0.4, limit=1.0, gated=False, passed=True),
    ]
    result = Calibrate.Result(
        error_code=Calibrate.Result.NONE, transform_valid=True, passed=True,
        quality=QualityReport(gates=gates, folds=5, fold_translation_max_m=0.0007,
                              scaled_min_singular_value=math.nan),
        message='ok',
    )
    result.camera_transform.header.frame_id = 'base_link'
    result.camera_transform.child_frame_id = 'camera_color_optical_frame'

    back = deserialize_message(serialize_message(result), Calibrate.Result)

    assert back.camera_transform.child_frame_id == 'camera_color_optical_frame'
    assert [g.name for g in back.quality.gates] == ['min_inliers', 'heldout_reprojection_rms']
    assert back.quality.gates[1].gated is False
    assert back.quality.fold_translation_max_m == 0.0007
    assert math.isnan(back.quality.scaled_min_singular_value)


def test_goal_and_feedback_round_trip():
    goal = Calibrate.Goal(mode=Calibrate.Goal.MODE_EYE_IN_HAND,
                          execution=Calibrate.Goal.EXECUTION_MANUAL, profile='bench')
    fb = Calibrate.Feedback(phase=Calibrate.Feedback.PHASE_MANUAL_CAPTURE, accepted=4,
                            rejected=2, last_rejection=Calibrate.Feedback.REJECT_BLURRY,
                            last_rejection_detail='laplacian variance 12 < 40')

    goal_back = deserialize_message(serialize_message(goal), Calibrate.Goal)
    fb_back = deserialize_message(serialize_message(fb), Calibrate.Feedback)

    assert (goal_back.mode, goal_back.execution, goal_back.profile) == (
        'eye_in_hand', 'manual', 'bench')
    assert (fb_back.phase, fb_back.accepted, fb_back.rejected) == ('manual_capture', 4, 2)
    assert fb_back.last_rejection == 'blurry'
    assert fb_back.last_rejection_detail == 'laplacian variance 12 < 40'
