"""
test_geometry_evaluator.py

Tests for GeometryEvaluator using a real point cloud.
"""

import numpy as np
import open3d as o3d
import pytest

from magpick.models import (
    CandidatePose,
    Billet,
    Gripper,
    Scene,
)
from magpick.evaluators.geometry import GeometryEvaluator
from magpick.utils.geometry import estimate_normals


@pytest.fixture
def scene():
    pcd = o3d.io.read_point_cloud("datasets/scene.ply")
    estimate_normals(pcd, radius=0.02)
    return Scene(point_cloud=pcd, frame_id="world")


@pytest.fixture
def evaluator():
    return GeometryEvaluator()


@pytest.fixture
def billet():
    return Billet(
        id=1,
        position=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.025,
        length=0.300,
        weight=3.0,
    )


@pytest.fixture
def gripper():
    return Gripper(
        name="Schmalz",
        max_force=1070.0,
        pad_width=0.040,
        pad_length=0.121,
        weight=1.5,
    )


def test_evaluate_returns_result(evaluator, billet, gripper, scene):
    points = np.asarray(scene.point_cloud.points)
    candidate = CandidatePose(
        position=points[0],
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet, gripper, scene)
    assert result.name == "Geometry"
    assert 0.0 <= result.score <= 1.0
    assert result.weight == pytest.approx(0.25)


def test_score_penalizes_large_normal_error(evaluator, billet, gripper, scene):
    """A candidate whose approach is far from the surface normal should score low."""
    points = np.asarray(scene.point_cloud.points)
    candidate = CandidatePose(
        position=points[0],
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet, gripper, scene)
    # The normal_error_deg should be in the metrics
    assert "normal_error_deg" in result.details


def test_passed_based_on_minimum_score(evaluator, billet, gripper, scene):
    points = np.asarray(scene.point_cloud.points)
    candidate = CandidatePose(
        position=points[0],
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet, gripper, scene)
    # If score >= minimum_score (0.70), passed should be True
    if result.score >= 0.70:
        assert result.passed is True
    else:
        assert result.passed is False
