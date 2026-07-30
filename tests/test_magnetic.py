"""
test_magnetic.py

Tests for MagneticEvaluator — the most physics-critical evaluator.
"""

import numpy as np
import pytest

from magpick.models import Billet, Gripper, RobotMotion, CandidatePose
from magpick.evaluators.magnetic import MagneticEvaluator
from magpick.config import config


@pytest.fixture
def evaluator():
    return MagneticEvaluator()


@pytest.fixture
def gripper():
    return Gripper(
        name="SGM-HP 40x121",
        max_force=1070.0,
        pad_width=0.040,
        pad_length=0.121,
        weight=1.5,
    )


@pytest.fixture
def steel_billet():
    return Billet(
        id=1,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.020,
        length=0.200,
        weight=2.5,
        material="forged_steel",
        surface="clean",
        air_gap=0.0,
    )


def test_weight_matches_config(evaluator):
    assert evaluator.cfg["weight"] == config["magnetic"]["weight"]


def test_forged_steel_clean_surface_high_score(evaluator, gripper, steel_billet):
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
    )
    assert result.score > 0.8
    assert result.passed is True
    assert result.details["material_factor"] == 1.0
    assert result.details["surface_factor"] == 1.0


def test_aluminium_always_fails(evaluator, gripper):
    billet = Billet(
        id=2,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.020,
        length=0.200,
        weight=2.5,
        material="aluminium",
        surface="clean",
        air_gap=0.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    assert result.details["material_factor"] == 0.0
    assert result.details["material_recognized"] is True
    assert result.passed is False
    assert result.score == 0.0


def test_unrecognized_material_fails_safe(evaluator, gripper):
    billet = Billet(
        id=3,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.020,
        length=0.200,
        weight=2.5,
        material="unobtainium",
        surface="clean",
        air_gap=0.0,
    )
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet,
        gripper,
        scene=None,
    )
    assert result.details["material_factor"] == 0.0
    assert result.details["material_recognized"] is False
    assert result.passed is False


def test_oily_surface_derates(evaluator, gripper, steel_billet):
    steel_billet.surface = "oily"
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
    )
    assert result.details["surface_factor"] == pytest.approx(0.80)
    assert result.details["surface_recognized"] is True


def test_air_gap_derating_applied(evaluator, gripper, steel_billet):
    steel_billet.air_gap = 1.0  # 1 mm additional gap
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
    )
    # With 1mm air gap, factor should be 0.70
    assert result.details["air_gap_derating"] == pytest.approx(0.70)
    assert result.details["total_air_gap_mm"] == pytest.approx(1.0)


def test_curvature_gap_computed_for_round_billet(evaluator, gripper, steel_billet):
    """For a 40mm dia billet on a 40mm wide pad, curvature gap is ~0."""
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
    )
    # 20mm radius billet, 40mm pad width — chord = pad_width/2 = 20mm
    # curvature_gap = r - sqrt(r^2 - (w/2)^2) = 20 - sqrt(400 - 400) = 0
    assert result.details["curvature_gap_mm"] == pytest.approx(0.0)


def test_dynamic_force_increases_required(evaluator, gripper, steel_billet):
    robot = RobotMotion(velocity=1.5, acceleration=3.0)
    result_static = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
        robot_motion=None,
    )
    result_dynamic = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
        robot_motion=robot,
    )
    assert result_dynamic.details["required_force"] > result_static.details["required_force"]
    assert result_dynamic.details["safety_factor"] < result_static.details["safety_factor"]


def test_score_thresholds_from_config(evaluator):
    thresholds = evaluator.cfg.get("score_thresholds", [])
    assert len(thresholds) > 0
    # Verify thresholds are in descending order
    sf_values = [t["min_safety_factor"] for t in thresholds]
    assert sf_values == sorted(sf_values, reverse=True)


def test_reason_message_on_failure(evaluator, gripper, steel_billet):
    steel_billet.weight = 100.0  # very heavy billet
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        steel_billet,
        gripper,
        scene=None,
    )
    assert result.passed is False
    assert "below" in result.reason.lower() or "safety factor" in result.reason.lower()
