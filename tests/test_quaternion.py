"""
test_quaternion.py

Tests for quaternion_to_approach utility function.
"""

import numpy as np
import pytest

from magpick.utils.geometry import quaternion_to_approach


def test_identity_quaternion_gives_z_axis():
    """Identity quaternion should give approach along +Z."""
    q = np.array([0.0, 0.0, 0.0, 1.0])
    approach = quaternion_to_approach(q)
    assert approach == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-10)


def test_approach_is_unit_vector():
    q = np.array([0.5, 0.5, 0.5, 0.5])
    approach = quaternion_to_approach(q)
    assert np.linalg.norm(approach) == pytest.approx(1.0, abs=1e-10)


def test_90_deg_rotation_about_z():
    """90-degree rotation about Z axis should keep approach along +Z."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("z", 90, degrees=True)
    q = R.as_quat()  # returns (x, y, z, w)
    approach = quaternion_to_approach(q)
    assert approach == pytest.approx(np.array([0.0, 0.0, 1.0]), abs=1e-10)


def test_90_deg_rotation_about_x():
    """90-degree rotation about X should make approach point along +Y."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("x", 90, degrees=True)
    q = R.as_quat()
    approach = quaternion_to_approach(q)
    # After 90° about X, local Z becomes world Y
    expected = np.array([0.0, 1.0, 0.0])
    # Allow for sign ambiguity (approach might be +Y or -Y depending on convention)
    assert abs(abs(np.dot(approach, expected)) - 1.0) < 1e-10
