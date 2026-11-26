"""Consensus, TF pairing and the capture record / observations YAML (§D2, §D5)."""

from handeye_autopilot.core import codes, observations
from handeye_autopilot.core import transforms as tf
from handeye_autopilot.core.capture import CaptureRecord, Frame, capture, pairwise_spread
from handeye_autopilot.core.config import CaptureConfig
from m1a_scenes import SPEC, build
import numpy as np
import pytest

CFG = CaptureConfig()
SCENE = SPEC['fixtures']['charuco_tilted']
META = {'mode': 'eye_to_hand',
        'frames': {'base_frame': 'base_link', 'tool_frame': 'tool0',
                   'camera_frame': 'camera_color_optical_frame'}}


def tool(k):
    return tf.make(np.eye(3), [0.4 + 0.001 * k, 0.0, 0.3])   # a distinct pose per frame


def frames(n=3, shifts=None, tools=True, blank=()):
    out, s = [], None
    for k in range(n):
        s = build(SCENE)
        if shifts is not None:
            s.objects[0].camera_T_plane = s.objects[0].camera_T_plane @ tf.make(
                np.eye(3), [shifts[k], 0, 0])
        img = s.render(seed=100 + k)
        if k in blank:
            img = np.full_like(img, 128)
        out.append(Frame(img, 10.0 + 0.033 * k, tool(k) if tools else None))
    return out, s


def test_kept_frame_is_one_real_frame_with_its_own_tool_pose():
    fs, s = frames()
    rec = capture(0, fs, s.board, s.camera, CFG)
    assert rec.accepted, rec.detail
    k = [f.stamp for f in fs].index(rec.stamp)
    np.testing.assert_array_equal(rec.tool_pose, fs[k].tool_pose)   # same frame's TF
    redo = capture(0, [fs[k]], s.board, s.camera, CaptureConfig(consensus_min_frames=1))
    np.testing.assert_array_equal(rec.image_points, redo.image_points)  # same frame's corners
    assert rec.metrics['frames_used'] == 3
    assert len(rec.ippe_poses) == 2 and rec.ippe_errors_px[0] <= rec.ippe_errors_px[1]


def test_consensus_uses_pairwise_disagreement():
    """Frames at -d, 0, +d: each is within d of the medoid, but two are 2d apart."""
    d = 0.6 * CFG.consensus_max_translation_frac * 0.55
    fs, s = frames(shifts=[-d, 0.0, d])
    rec = capture(1, fs, s.board, s.camera, CFG)
    assert rec.reject == codes.CONSENSUS, rec.detail
    assert rec.image_points is not None        # the processed frame is kept in the record


def test_consensus_rejects_a_shifted_frame():
    fs, s = frames(shifts=[0, 0, SPEC['gate_scenes']['consensus_shift_m']])
    assert capture(1, fs, s.board, s.camera, CFG).reject == codes.CONSENSUS


def test_failed_frame_limit():
    fs, s = frames(n=4, blank=(3,))
    assert capture(2, fs, s.board, s.camera, CFG).accepted     # 1 failed frame allowed
    fs, s = frames(n=4, blank=(2, 3))
    rec = capture(2, fs, s.board, s.camera, CFG)
    assert rec.reject == codes.NO_BOARD and '2 of 4' in rec.detail


def test_frames_without_tf_reject_the_capture_and_fields_are_null():
    fs, s = frames(tools=False)
    rec = capture(3, fs, s.board, s.camera, CFG)
    assert rec.reject == codes.TF_UNAVAILABLE
    d = rec.to_dict()
    assert d['ids'] == [] and d['image_points'] is None and d['tool_pose'] is None
    assert d['camera_T_board'] is None and d['ippe_poses'] is None


def test_mixed_tf_rejection_records_the_missing_frame_stamp():
    fs, s = frames()
    fs[1].tool_pose = None
    fs[2].tool_pose = None
    rec = capture(5, fs, s.board, s.camera, CFG)
    assert rec.reject == codes.TF_UNAVAILABLE and rec.stamp == fs[1].stamp


def test_no_board_keeps_empty_ids_and_null_points():
    fs, s = frames(blank=(0, 1, 2))
    d = capture(4, fs, s.board, s.camera, CFG).to_dict()
    assert d['reject'] == codes.NO_BOARD
    assert d['ids'] == [] and d['object_points'] is None and d['stamp'] is not None


def test_record_and_observations_round_trip(tmp_path):
    fs, s = frames()
    good = capture(0, fs, s.board, s.camera, CFG)
    bad = capture(1, frames(tools=False)[0], s.board, s.camera, CFG)
    path = tmp_path / 'observations.yaml'
    observations.save(path, [good, bad], s.board, META)
    recs, board2, meta = observations.read(path)
    assert board2 == s.board and meta == META
    a, b = recs
    np.testing.assert_array_equal(a.ids, good.ids)
    np.testing.assert_allclose(a.image_points, good.image_points)
    np.testing.assert_allclose(a.camera_T_board, good.camera_T_board)
    assert a.accepted and not b.accepted and b.reject == codes.TF_UNAVAILABLE


@pytest.mark.parametrize('text, match', [
    ('schema_version: 99\ncaptures: []\n', 'schema_version 99'),
    ("schema_version: '1'\ncaptures: []\n", 'integer'),
    ('captures: []\n', 'no schema_version'),
    ('schema_version: 1\nmetadata: {mode: sideways}\ncaptures: []\n', 'mode'),
    ('schema_version: 1\nmetadata: {mode: eye_in_hand}\ncaptures: []\n', 'frames'),
])
def test_observations_loader_refuses_bad_files(text, match):
    with pytest.raises(observations.ObservationsError, match=match):
        observations.load(text)


def test_loader_rejects_malformed_values():
    fs, s = frames()
    d = capture(0, fs, s.board, s.camera, CFG).to_dict()
    with pytest.raises(ValueError, match='true/false'):
        CaptureRecord.from_dict(dict(d, accepted='false'))
    with pytest.raises(ValueError, match='null IPPE'):
        CaptureRecord.from_dict(dict(d, ippe_poses=[d['ippe_poses'][0], None]))
    bad_meta = dict(META, frames=dict(META['frames'], tool_frame=7))
    with pytest.raises(observations.ObservationsError, match='frames'):
        observations.dump([], s.board, bad_meta)


@pytest.mark.parametrize('field, value', [
    ('tool_pose', None), ('stamp', None), ('ippe_poses', 'one'), ('image_points', None)])
def test_accepted_record_must_have_solver_inputs(field, value):
    fs, s = frames()
    d = capture(0, fs, s.board, s.camera, CFG).to_dict()
    d[field] = d['ippe_poses'][:1] if value == 'one' else value
    with pytest.raises(ValueError, match='missing solver inputs'):
        CaptureRecord.from_dict(d)


def test_pairwise_spread():
    a, b = tf.make(np.eye(3), [0, 0, 1]), tf.make(np.eye(3), [0.002, 0, 1])
    assert pairwise_spread([a, b])[0] == pytest.approx(0.002)
    assert pairwise_spread([a]) == (0.0, 0.0)
