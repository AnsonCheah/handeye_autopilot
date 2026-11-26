"""Downstream check: the generated interfaces import and construct (M0 done-when)."""

from handeye_autopilot_interfaces.action import Calibrate
from handeye_autopilot_interfaces.msg import GateResult, QualityReport


def test_calibrate_goal_result_feedback_construct():
    goal = Calibrate.Goal(
        mode=Calibrate.Goal.MODE_EYE_TO_HAND,
        execution=Calibrate.Goal.EXECUTION_MOVEIT,
    )
    assert goal.mode == 'eye_to_hand'
    assert goal.profile == ''

    result = Calibrate.Result(error_code=Calibrate.Result.MOTION_FAULT)
    assert result.error_code == 6
    assert not result.passed and not result.transform_valid
    assert result.camera_transform.transform.rotation.w == 1.0

    fb = Calibrate.Feedback(
        phase=Calibrate.Feedback.PHASE_BOOTSTRAP, accepted=3, rejected=1,
        last_rejection=Calibrate.Feedback.REJECT_AMBIGUOUS_POSE,
    )
    assert fb.accepted == 3


def test_error_codes_are_unique():
    codes = {
        name: getattr(Calibrate.Result, name)
        for name in dir(Calibrate.Result)
        if name.isupper() and isinstance(getattr(Calibrate.Result, name), int)
    }
    assert len(codes) == 11
    assert 'BUSY' not in codes  # admission rejections carry no result (§5)
    assert len(set(codes.values())) == len(codes)


def test_quality_report_construct():
    gate = GateResult(
        name='fold_translation_max', group=GateResult.GROUP_FOLD_SENSITIVITY,
        unit='m', comparison='<=', value=0.001, limit=0.002, gated=True, passed=True,
    )
    report = QualityReport(gates=[gate], folds=5, fold_translation_max_m=0.001)
    assert report.gates[0].group == 'fold_sensitivity'
    assert report.folds == 5
