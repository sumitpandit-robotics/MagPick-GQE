"""
test_angle.py

Tests for the angle_between utility function.
"""

import numpy as np
import pytest

from magpick.utils.geometry import angle_between


def test_angle_zero_degrees():
    assert angle_between([0, 0, 1], [0, 0, 1]) == pytest.approx(0.0, abs=1e-10)


def test_angle_90_degrees():
    assert angle_between([1, 0, 0], [0, 1, 0]) == pytest.approx(90.0, abs=1e-10)


def test_angle_180_degrees():
    assert angle_between([0, 0, 1], [0, 0, -1]) == pytest.approx(180.0, abs=1e-10)


def test_angle_45_degrees():
    v1 = np.array([1, 0, 0])
    v2 = np.array([1, 1, 0]) / np.sqrt(2)
    assert angle_between(v1, v2) == pytest.approx(45.0, abs=1e-10)


def test_angle_symmetric():
    """angle(v1, v2) should equal angle(v2, v1)."""
    a = angle_between([1, 2, 3], [4, 5, 6])
    b = angle_between([4, 5, 6], [1, 2, 3])
    assert a == pytest.approx(b, abs=1e-10)


def test_angle_normalised_input():
    """Should work with both normalised and unnormalised vectors."""
    a = angle_between([1, 0, 0], [0, 1, 0])
    b = angle_between([5, 0, 0], [0, 3, 0])
    assert a == pytest.approx(b, abs=1e-10)
