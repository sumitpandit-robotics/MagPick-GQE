"""
test_contact_area.py

Tests for ContactAreaEvaluator — pad-to-billet coverage.
"""

import numpy as np
import pytest

from magpick.models import Billet, Gripper, CandidatePose
from magpick.evaluators.contact_area import ContactAreaEvaluator
from magpick.config import config


@pytest.fixture
def evaluator():
    return ContactAreaEvaluator()


@pytest.fixture
def gripper():
    return Gripper(
        name="SGM-HP 40x121",
        max_force=1070.0,
        pad_width=0.040,
        pad_length=0.121,
        weight=1.5,
    )


def test_weight_matches_config(evaluator):
    assert evaluator.cfg["weight"] == config["contact"]["weight"]


def test_full_coverage_billet_larger_than_pad(evaluator, gripper):
    """A billet larger than the pad should give coverage_ratio = 1.0."""
    billet = Billet(
        id=1,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.050,    # 100mm dia > 40mm pad width
        length=0.300,    # 300mm > 121mm pad length
        weight=5.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    assert result.details["width_coverage"] == pytest.approx(1.0)
    assert result.details["length_coverage"] == pytest.approx(1.0)
    assert result.details["coverage_ratio"] == pytest.approx(1.0)


def test_partial_coverage_narrow_billet(evaluator, gripper):
    """A billet narrower than the pad width should limit coverage."""
    billet = Billet(
        id=2,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.015,    # 30mm dia < 40mm pad width
        length=0.300,
        weight=2.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    assert result.details["width_coverage"] < 1.0
    assert result.details["coverage_ratio"] == result.details["width_coverage"]


def test_partial_coverage_short_billet(evaluator, gripper):
    """A billet shorter than the pad length should limit coverage."""
    billet = Billet(
        id=3,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.050,    # 100mm dia > 40mm pad width
        length=0.080,    # 80mm < 121mm pad length
        weight=3.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    assert result.details["length_coverage"] < 1.0
    assert result.details["coverage_ratio"] == result.details["length_coverage"]


def test_score_equals_contact_factor(evaluator, gripper):
    billet = Billet(
        id=4,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.020,
        length=0.200,
        weight=2.5,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    assert result.score == pytest.approx(result.details["contact_factor"])


def test_metrics_contain_expected_keys(evaluator, gripper):
    billet = Billet(
        id=5,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.020,
        length=0.200,
        weight=2.5,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    for key in [
        "billet_diameter", "billet_length", "pad_width", "pad_length",
        "width_coverage", "length_coverage", "coverage_ratio", "contact_factor",
    ]:
        assert key in result.details
