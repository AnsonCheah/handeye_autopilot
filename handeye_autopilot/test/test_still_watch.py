"""StillWatch (node.py): the joint evidence an INVALID_MOTION_PLAN skip needs
(01-architecture §3.2). Driven directly with JointState messages."""

from handeye_autopilot.ros.node import STILL_MAX_GAP, StillWatch
from sensor_msgs.msg import JointState

REF = {'a': 0.0, 'b': 1.0}
TOL = 0.01


def _msg(stamp, **pos):
    m = JointState(name=list(pos), position=[float(v) for v in pos.values()])
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    return m


def _watch():
    return StillWatch(dict(REF), TOL, last=0.0, start_stamp=100.0)


def _feed(w, samples):
    for now, stamp, pos in samples:
        m = _msg(stamp, **pos)
        w.sample(now, stamp, m)


def test_still_with_steady_full_samples():
    w = _watch()
    _feed(w, [(0.02 * i, 100.0 + 0.02 * i, REF) for i in range(1, 20)])
    w.end(0.4, 100.0 + 0.4)
    assert w.still(), w.why()


def test_a_partial_message_that_moved_counts():
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.04, 100.04, {'a': 0.3}), (0.06, 100.06, REF)])
    w.end(0.08, 100.0 + 0.08)
    assert not w.still() and 'left the start' in w.why()


def test_a_round_trip_between_full_samples_counts():
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.04, 100.04, {'a': 0.3, 'b': 1.0}), (0.06, 100.06, REF)])
    w.end(0.08, 100.0 + 0.08)
    assert not w.still()


def test_messages_stamped_before_the_start_are_not_evidence():
    w = _watch()
    _feed(w, [(0.02, 99.0, REF), (0.04, 99.5, {'a': 0.3, 'b': 1.0})])
    w.end(0.06, 100.0 + 0.06)
    assert not w.still() and w.why() == 'no joint samples'      # ignored, both ways


def test_a_gap_between_samples_or_at_the_end_is_not_evidence():
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.02 + STILL_MAX_GAP + 0.05, 100.3, REF)])
    w.end(0.4, 100.0 + 0.4)
    assert not w.still() and 'gap' in w.why()
    w = _watch()
    _feed(w, [(0.02, 100.02, REF)])
    end = 0.02 + STILL_MAX_GAP + 0.05          # nothing since: the end counts too
    w.end(end, 100.0 + end)
    assert not w.still()


def test_re_sent_messages_are_not_new_samples():
    w = _watch()
    _feed(w, [(0.02 * i, 100.02, REF) for i in range(1, 10)])     # one message, nine times
    assert w.samples == 1 and w.last == 0.02


def test_the_end_gap_counts_in_stamps_too():
    """Receive time looks dense (late partial messages keep the callback busy)
    but the last full sample is old in the stamps' clock (e.g. sim time 4x)."""
    w = _watch()
    _feed(w, [(0.02, 100.02, REF)])
    w.end(0.1, 100.7)
    assert not w.still() and 'gap' in w.why()


def test_a_stale_clock_does_not_hide_the_end_gap():
    """The node's clock stalls at the last full sample, but later (partial)
    messages carry newer stamps: those show the gap."""
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.05, 100.7, {'a': 0.0})])
    w.end(0.06, 100.02)
    assert not w.still() and 'gap' in w.why()


def test_a_newer_message_about_other_joints_shows_the_end_gap():
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.05, 100.7, {'gripper': 0.04})])
    w.end(0.06, 100.02)                         # node clock stalled
    assert not w.still() and 'gap' in w.why()


def test_an_older_arm_message_after_a_newer_one_does_not_hide_the_gap():
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.04, 100.7, {'gripper': 0.04}), (0.05, 100.03, REF)])
    w.end(0.06, 100.03)                         # the newest stamp, not the last one, counts
    assert not w.still() and 'gap' in w.why()


def test_an_out_of_order_sample_does_not_bridge_a_gap():
    w = _watch()
    _feed(w, [(0.02, 100.10, REF), (0.4, 100.05, REF), (0.42, 100.12, REF)])
    assert w.samples == 2                       # the older stamp is not a new sample
    w.end(0.44, 100.14)
    assert not w.still() and 'gap' in w.why()   # 0.38 s receive gap stays


def test_a_re_sent_message_does_not_bridge_a_gap():
    """The same stationary message delivered again and again (stamp unchanged)
    while the real samples stop: the stamps show the gap."""
    w = _watch()
    _feed(w, [(0.02, 100.02, REF)] + [(0.02 + 0.1 * i, 100.02, REF) for i in range(1, 8)]
          + [(0.8, 100.8, REF)])
    w.end(0.82, 100.0 + 0.82)
    assert not w.still() and 'gap' in w.why()


def test_stamps_must_also_be_dense():
    w = _watch()
    _feed(w, [(0.02, 100.02, REF), (0.04, 100.6, REF), (0.06, 100.62, REF)])
    w.end(0.08, 100.0 + 0.08)
    assert not w.still()


def test_partial_messages_alone_do_not_cover_the_time():
    w = _watch()
    _feed(w, [(0.02 * i, 100.0 + 0.02 * i, {'a': 0.0}) for i in range(1, 20)])
    w.end(0.4, 100.0 + 0.4)
    assert not w.still()


def test_non_finite_values_count_as_moved():
    w = _watch()
    _feed(w, [(0.02, 100.02, {'a': float('nan'), 'b': 1.0})])
    assert w.moved
