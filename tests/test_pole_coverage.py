"""
test_pole_coverage.py

Tests for PoleCoverageEvaluator.
"""

import numpy as np
import pytest

from magpick.models import Billet, Gripper, CandidatePose
from magpick.evaluators.pole_coverage import PoleCoverageEvaluator
from magpick.gripper_profile import GripperProfile, PoleLayout


@pytest.fixture
def evaluator():
    return PoleCoverageEvaluator()


@pytest.fixture
def gripper_with_poles():
    """A gripper with a known pole layout."""
    return Gripper(
        name="Test Gripper",
        max_force=1070.0,
        pad_width=0.040,
        pad_length=0.121,
        weight=1.5,
        footprint_shape="rectangle",
    )


@pytest.fixture
def pole_layout_8():
    """8 poles in 2 rows of 4, within the 40mm × 121mm pad footprint."""
    return PoleLayout(
        num_poles=8,
        pole_positions_m=[
            (-0.045, -0.015), (-0.015, -0.015), (0.015, -0.015), (0.045, -0.015),
            (-0.045,  0.015), (-0.015,  0.015), (0.015,  0.015), (0.045,  0.015),
        ],
        pole_diameter_m=0.018,
    )


def test_weight_matches_config(evaluator):
    from magpick.config import config
    assert evaluator.cfg["weight"] == config["pole_coverage"]["weight"]


def test_no_pole_data_returns_full_coverage(evaluator, gripper_with_poles):
    """Without pole layout, should default to coverage=1.0."""
    billet = Billet(
        id=1, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=2.5,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper_with_poles, scene=None,
    )
    assert result.score == 1.0


def test_large_billet_covers_all_poles(evaluator, gripper_with_poles, pole_layout_8):
    """A billet much larger than the pad should cover all poles."""
    gripper_with_poles.pole_layout = pole_layout_8
    # Use a billet with radius large enough that all pole positions
    # fall within the circular cross-section.
    # Max pole distance from center: sqrt(0.045² + 0.025²) ≈ 0.0515m
    billet = Billet(
        id=2, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.060, length=0.300, weight=5.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper_with_poles, scene=None,
    )
    assert result.details["poles_in_contact"] == 8
    assert result.score == 1.0


def test_narrow_billet_covers_fewer_poles(evaluator, gripper_with_poles, pole_layout_8):
    """A narrow billet should cover fewer poles than the full layout."""
    gripper_with_poles.pole_layout = pole_layout_8
    # Use a billet with radius smaller than the outer pole positions.
    # Inner poles at (±0.015, ±0.015): distance ≈ 21mm
    # Outer poles at (±0.045, ±0.015): distance ≈ 47mm
    # A radius of 30mm should cover only the inner 4 poles.
    billet = Billet(
        id=3, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.030, length=0.200, weight=1.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper_with_poles, scene=None,
    )
    assert result.details["poles_in_contact"] < 8
    assert result.score < 1.0
