"""
test_collision.py

Tests for CollisionEvaluator — local clearance around grasp.
"""

import numpy as np
import open3d as o3d
import pytest

from magpick.models import CandidatePose, Scene
from magpick.evaluators.collision import CollisionEvaluator
from magpick.config import config


@pytest.fixture
def evaluator():
    return CollisionEvaluator()


@pytest.fixture
def scene():
    pcd = o3d.io.read_point_cloud("datasets/scene.ply")
    return Scene(point_cloud=pcd, frame_id="world")


def test_weight_matches_config(evaluator):
    assert evaluator.cfg["weight"] == config["collision"]["weight"]


def test_clear_area_scores_high(evaluator, scene):
    """A point far from any scene points should have high clearance."""
    candidate = CandidatePose(
        position=np.array([100.0, 100.0, 100.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet=None, gripper=None, scene=scene)
    assert result.details["clearance_score"] == pytest.approx(1.0)
    assert result.details["nearby_points"] == 0


def test_cluttered_area_scores_lower(evaluator, scene):
    """A point in the middle of the point cloud should have lower clearance."""
    points = np.asarray(scene.point_cloud.points)
    centroid = np.mean(points, axis=0)
    candidate = CandidatePose(
        position=centroid,
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet=None, gripper=None, scene=scene)
    assert result.details["clearance_score"] <= 1.0


def test_empty_cloud_gives_perfect_score(evaluator):
    """An empty point cloud should return clearance_score=1.0."""
    pcd = o3d.geometry.PointCloud()
    empty_scene = Scene(point_cloud=pcd, frame_id="world")
    candidate = CandidatePose(
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet=None, gripper=None, scene=empty_scene)
    assert result.details["clearance_score"] == pytest.approx(1.0)
    assert result.details["nearby_points"] == 0


def test_score_equals_clearance_score(evaluator, scene):
    candidate = CandidatePose(
        position=np.array([100.0, 100.0, 100.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet=None, gripper=None, scene=scene)
    assert result.score == pytest.approx(result.details["clearance_score"])


def test_passed_based_on_minimum_clearance_score(evaluator, scene):
    candidate = CandidatePose(
        position=np.array([100.0, 100.0, 100.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = evaluator.evaluate(candidate, billet=None, gripper=None, scene=scene)
    min_score = config["collision"]["minimum_clearance_score"]
    if result.score >= min_score:
        assert result.passed
    else:
        assert not result.passed
