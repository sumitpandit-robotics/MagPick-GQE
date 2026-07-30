"""
test_robot_dynamics.py

Tests for RobotDynamicsEvaluator.
"""

import numpy as np
import pytest

from magpick.models import Billet, Gripper, CandidatePose, RobotMotion
from magpick.evaluators.robot_dynamics import RobotDynamicsEvaluator


@pytest.fixture
def evaluator():
    return RobotDynamicsEvaluator()


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
def billet():
    return Billet(
        id=1, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=2.5,
    )


def test_weight_matches_config(evaluator):
    from magpick.config import config
    assert evaluator.cfg["weight"] == config["robot_dynamics"]["weight"]


def test_static_load_passes(evaluator, gripper, billet):
    """A static load (no robot motion) should have high safety factor."""
    result = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper, scene=None, robot_motion=None,
    )
    assert result.details["dynamic_safety_factor"] > 1.0
    assert result.passed


def test_dynamic_load_reduces_safety(evaluator, gripper, billet):
    """Adding robot motion should reduce the safety factor."""
    robot = RobotMotion(velocity=1.5, acceleration=3.0)
    result_static = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper, scene=None, robot_motion=None,
    )
    result_dynamic = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper, scene=None, robot_motion=robot,
    )
    assert result_dynamic.details["dynamic_safety_factor"] < result_static.details["dynamic_safety_factor"]
    assert result_dynamic.details["dynamic_force_N"] > result_static.details["dynamic_force_N"]


def test_heavy_billet_increases_force(evaluator, gripper):
    """A heavier billet should produce higher dynamic forces."""
    light = Billet(
        id=1, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=1.0,
    )
    heavy = Billet(
        id=2, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.020, length=0.200, weight=10.0,
    )
    robot = RobotMotion(velocity=1.5, acceleration=3.0)
    r_light = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        light, gripper, scene=None, robot_motion=robot,
    )
    r_heavy = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        heavy, gripper, scene=None, robot_motion=robot,
    )
    assert r_heavy.details["total_force_N"] > r_light.details["total_force_N"]
    assert r_heavy.details["dynamic_safety_factor"] < r_light.details["dynamic_safety_factor"]


def test_emergency_stop_considered(evaluator, gripper, billet):
    """Emergency-stop deceleration should be considered."""
    robot_normal = RobotMotion(velocity=1.5, acceleration=3.0, emergency_stop_acceleration=0.0)
    robot_estop = RobotMotion(velocity=1.5, acceleration=3.0, emergency_stop_acceleration=20.0)
    r_normal = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper, scene=None, robot_motion=robot_normal,
    )
    r_estop = evaluator.evaluate(
        CandidatePose(position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0])),
        billet, gripper, scene=None, robot_motion=robot_estop,
    )
    # Emergency stop should produce higher dynamic force
    assert r_estop.details["dynamic_force_N"] >= r_normal.details["dynamic_force_N"]
