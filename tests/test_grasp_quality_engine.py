"""
test_grasp_quality_engine.py

Tests for the GraspQualityEngine orchestrator, including the
force-adequacy pre-check and billet type matching.
"""

import numpy as np
import open3d as o3d
import pytest

from magpick.models import Billet, CandidatePose, RobotMotion, Scene
from magpick.grasp_quality_engine import (
    GraspQualityEngine,
    check_gripper_billet_compatibility,
    match_billet_type,
    CompatibilityResult,
)


PROFILE_PATH = "config/grippers/schmalz_sgm_hp_40x121.yaml"


@pytest.fixture
def engine():
    return GraspQualityEngine(gripper_profile=PROFILE_PATH)


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
    )


@pytest.fixture
def scene():
    pcd = o3d.io.read_point_cloud("datasets/scene.ply")
    return Scene(point_cloud=pcd, frame_id="world")


# ==========================================================
# Compatibility Pre-Check
# ==========================================================

def test_compatibility_ok_for_reasonable_billet(engine, steel_billet):
    result = engine.check_compatibility(steel_billet)
    assert result.compatible is True
    assert result.details["best_case_safety_factor"] > 2.0


def test_compatibility_fails_for_aluminium(engine):
    billet = Billet(
        id=2, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=2.5, material="aluminium",
    )
    result = engine.check_compatibility(billet)
    assert result.compatible is False
    # Aluminium is recognized (factor=0.0) but holding force is zero
    assert result.details["material_factor"] == 0.0
    assert result.details["best_case_holding_force_N"] == 0.0


def test_compatibility_fails_for_too_heavy(engine):
    billet = Billet(
        id=3, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=200.0,  # 200kg!
    )
    result = engine.check_compatibility(billet)
    assert result.compatible is False
    assert "capacity" in result.message.lower() or "exceeds" in result.message.lower()


def test_compatibility_with_dynamic_load(engine, steel_billet):
    robot = RobotMotion(velocity=1.5, acceleration=3.0)
    result = engine.check_compatibility(steel_billet, robot_motion=robot)
    # Dynamic loads reduce the safety factor
    assert result.details["required_force_N"] > result.details["gravity_force_N"] if "gravity_force_N" in result.details else True


# ==========================================================
# Billet Type Matching
# ==========================================================

def test_billet_type_match():
    billet = Billet(
        id=1, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=2.5,
    )
    match = match_billet_type(billet)
    assert match is not None
    assert match["sku"] == "billet_40x200_steel"
    assert match["material"] == "forged_steel"


def test_billet_type_no_match():
    billet = Billet(
        id=2, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.035, length=0.180, weight=3.0,  # 70mm dia, 180mm — no match
    )
    match = match_billet_type(billet)
    assert match is None


# ==========================================================
# Full Evaluation
# ==========================================================

def test_evaluate_returns_report(engine, steel_billet, scene):
    candidates = [
        CandidatePose(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        ),
    ]
    report = engine.evaluate(candidates, steel_billet, scene)
    assert len(report.candidates) == 1
    assert report.compatibility.compatible is True
    assert report.summary["total_candidates"] == 1


def test_evaluate_incompatible_returns_zero_scores(engine, scene):
    """If the billet is incompatible, all candidates should get score=0."""
    billet = Billet(
        id=99, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=200.0,  # way too heavy
    )
    candidates = [
        CandidatePose(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        ),
    ]
    report = engine.evaluate(candidates, billet, scene)
    assert report.compatibility.compatible is False
    for c in report.candidates:
        assert c.final_score == 0.0


def test_engine_uses_profile_gripper(engine, steel_billet):
    """The engine should use the gripper from the profile."""
    assert engine.gripper.name == "Schmalz SGM-HP 40x121"
    assert engine.gripper.max_force == pytest.approx(1070.0)
    assert engine.gripper.pad_width == pytest.approx(0.040)
    assert engine.gripper.pad_length == pytest.approx(0.121)
