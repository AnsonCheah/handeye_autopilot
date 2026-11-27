"""Settle rule (01-architecture §8) with scripted TF samples."""

import math

import numpy as np
import pytest

pytest.importorskip('tf2_ros')

from handeye_autopilot.ros import tf_tools  # noqa: E402


def run_settle(monkeypatch, samples, window=0.5, timeout=0.3):
    """samples: list of (stamp, x) returned by successive latest() calls."""
    it = iter(samples)
    last = [samples[-1]]

    def latest(buffer, base, tool):
        try:
            last[0] = next(it)
        except StopIteration:
            pass
        T = np.eye(4)
        T[0, 3] = last[0][1]
        return T, last[0][0]
    monkeypatch.setattr(tf_tools, 'latest', latest)
    return tf_tools.settle(None, 'b', 't', lambda: 0.0, window, 0.001, math.radians(0.1),
                           timeout, poll=0.0)


def test_rounding_at_the_window_edge_does_not_crash(monkeypatch):
    s = run_settle(monkeypatch, [(0.105, 0), (0.305, 0), (0.605, 0)])
    assert isinstance(s, tf_tools.Settled)


def test_stationary_5hz_qualifies(monkeypatch):
    s = run_settle(monkeypatch, [(0.2 * k, 0.0) for k in range(1, 8)])
    assert s.ok


def test_two_samples_far_apart_do_not_qualify(monkeypatch):
    s = run_settle(monkeypatch, [(0.1, 0.0), (0.9, 0.0)])
    assert not s.ok


def test_motion_inside_the_window_does_not_qualify(monkeypatch):
    s = run_settle(monkeypatch, [(0.1 * k, 0.01 * (k % 2)) for k in range(1, 12)])
    assert not s.ok


def test_stale_samples_are_frozen(monkeypatch):
    s = run_settle(monkeypatch, [(-1.0, 0.0)] * 5)
    assert not s.ok and 'frozen' in s.reason
