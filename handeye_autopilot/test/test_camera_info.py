"""CameraInfo validation (01-architecture §7)."""

import numpy as np
import pytest

pytest.importorskip('image_geometry')

from handeye_autopilot.ros.camera_input import CameraInfoError, camera_model, same_info  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402


def info(**over):
    m = CameraInfo()
    m.header.frame_id = 'cam'
    m.width, m.height = 640, 480
    m.distortion_model = 'plumb_bob'
    m.d = [0.1, -0.05, 0.0, 0.0, 0.0]
    m.k = [600.0, 0, 320, 0, 600.0, 240, 0, 0, 1]
    m.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    m.p = [600.0, 0, 320, 0, 0, 600.0, 240, 0, 0, 0, 1, 0]
    for k, v in over.items():
        setattr(m, k, v)
    return m


def test_raw_and_rectified_models():
    raw = camera_model(info(), rectified=False)
    assert np.allclose(raw.D, [0.1, -0.05, 0, 0, 0])
    rect = camera_model(info(), rectified=True)
    assert not np.any(rect.D) and rect.rectified


@pytest.mark.parametrize('over,rectified', [
    (dict(k=[0.0] * 9), False),                             # uncalibrated
    (dict(binning_x=2), False),
    (dict(distortion_model='equidistant', d=[0.1, 0, 0, 0]), False),
    (dict(r=[0.0] * 9), True),                              # rectified needs R = I
    (dict(r=[0.0, -1, 0, 1, 0, 0, 0, 0, 1]), True),         # stereo R
])
def test_refused(over, rectified):
    with pytest.raises(CameraInfoError):
        camera_model(info(**over), rectified=rectified)


def test_roi_refused():
    m = info()
    m.roi.x_offset, m.roi.width, m.roi.height = 10, 320, 240
    with pytest.raises(CameraInfoError):
        camera_model(m, rectified=False)


def test_same_info_compares_everything_and_never_raises():
    a = info()
    assert same_info(a, info())
    assert not same_info(a, info(d=[0.1, -0.05, 0, 0, 0, 0, 0, 0],
                                 distortion_model='plumb_bob'))     # length differs
    assert not same_info(a, info(r=[0.0, -1, 0, 1, 0, 0, 0, 0, 1]))
    assert not same_info(a, info(binning_x=2))


def test_empty_distortion_is_a_valid_raw_camera():
    m = camera_model(info(d=[]), rectified=False)
    assert m.D.size == 5 and not np.any(m.D)
