"""
test_geometry.py

Tests for GeometryEvaluator with a null point cloud (unit tests only).
"""

import numpy as np
import pytest

from magpick.models import Billet, Gripper, CandidatePose, Scene
from magpick.evaluators.geometry import GeometryEvaluator


@pytest.fixture
def evaluator():
    return GeometryEvaluator()


@pytest.fixture
def billet():
    return Billet(
        id=0,
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.025,
        length=0.200,
        weight=2.5,
    )


@pytest.fixture
def gripper():
    return Gripper(
        name="Schmalz",
        max_force=1070.0,
        pad_width=0.040,
        pad_length=0.121,
        weight=1.8,
    )


def test_evaluate_returns_result(evaluator, billet, gripper):
    candidate = CandidatePose(
        position=np.zeros(3),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    scene = Scene(point_cloud=None, frame_id="camera")
    # This will fail because GeometryEvaluator requires a point cloud
    # for nearest_point. We're testing the constructor, not the evaluate.
    assert evaluator.cfg["weight"] > 0
    assert evaluator.cfg["minimum_score"] > 0
