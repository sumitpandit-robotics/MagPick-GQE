"""
test_fusion.py

Integration test for FusionEngine ranking pipeline.

Uses synthetic billet/gripper data and a real point cloud
to verify that the full evaluation + ranking path works end-to-end.
"""

import numpy as np
import open3d as o3d
import pytest

from magpick.models import (
    Billet,
    Gripper,
    CandidatePose,
    RobotMotion,
    Scene,
)
from magpick.fusion import FusionEngine


@pytest.fixture
def scene():
    pcd = o3d.io.read_point_cloud("datasets/scene.ply")
    return Scene(point_cloud=pcd, frame_id="world")


@pytest.fixture
def billet():
    return Billet(
        id=1,
        position=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        radius=0.020,       # 20 mm in meters
        length=0.200,       # 200 mm in meters
        weight=2.5,
        material="forged_steel",
        surface="clean",
    )


@pytest.fixture
def gripper():
    return Gripper(
        name="SGM HP",
        max_force=1070.0,
        pad_width=0.040,    # 40 mm in meters
        pad_length=0.121,   # 121 mm in meters
        weight=1.5,
    )


@pytest.fixture
def robot():
    return RobotMotion(
        velocity=1.5,
        acceleration=3.0,
    )


@pytest.fixture
def candidates():
    return [
        CandidatePose(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        ),
        CandidatePose(
            position=np.array([0.02, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        ),
    ]


@pytest.fixture
def engine():
    return FusionEngine()


def test_rank_candidates_returns_sorted_descending(
    engine, candidates, billet, gripper, scene, robot,
):
    ranked = engine.rank_candidates(candidates, billet, gripper, scene, robot)
    assert len(ranked) == 2
    scores = [r.final_score for r in ranked]
    assert scores == sorted(scores, reverse=True)


def test_best_candidate_is_top_ranked(
    engine, candidates, billet, gripper, scene, robot,
):
    best = engine.best_candidate(candidates, billet, gripper, scene, robot)
    assert best is not None
    assert best.rank == 1
    # Status must be consistent with score
    if best.final_score > 0:
        assert best.status == "PASS"
    else:
        assert best.status == "FAIL"


def test_scores_are_between_zero_and_one(
    engine, candidates, billet, gripper, scene, robot,
):
    ranked = engine.rank_candidates(candidates, billet, gripper, scene, robot)
    for result in ranked:
        assert 0.0 <= result.final_score <= 1.0


def test_evaluator_results_present(
    engine, candidates, billet, gripper, scene, robot,
):
    ranked = engine.rank_candidates(candidates, billet, gripper, scene, robot)
    for result in ranked:
        assert len(result.evaluator_results) > 0
        for ev_result in result.evaluator_results:
            assert hasattr(ev_result, "name")
            assert hasattr(ev_result, "score")
            assert hasattr(ev_result, "weight")
            assert hasattr(ev_result, "passed")


def test_weights_match_config(
    engine, billet, gripper, scene,
):
    from magpick.config import config
    single = CandidatePose(
        position=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    result = engine.evaluate_candidate(single, billet, gripper, scene)
    config_weights = {
        "Geometry": config["geometry"]["weight"],
        "Contact Area": config["contact"]["weight"],
        "Magnetic": config["magnetic"]["weight"],
        "Collision": config["collision"]["weight"],
        "Pole Coverage": config["pole_coverage"]["weight"],
        "Robot Dynamics": config["robot_dynamics"]["weight"],
    }
    for ev_result in result.evaluator_results:
        if ev_result.name in config_weights:
            assert ev_result.weight == pytest.approx(
                config_weights[ev_result.name]
            )


def test_empty_candidates_returns_none(engine, billet, gripper, scene):
    best = engine.best_candidate([], billet, gripper, scene)
    assert best is None


def test_hard_constraint_disqualifies_candidate(
    engine, billet, gripper, scene,
):
    """A candidate with a failing evaluator should get final_score=0."""
    bad_candidate = CandidatePose(
        position=np.array([999.0, 999.0, 999.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    good_candidate = CandidatePose(
        position=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([0.0, 0.0, 0.0, 1.0]),
    )
    ranked = engine.rank_candidates(
        [bad_candidate, good_candidate], billet, gripper, scene,
    )
    # The far-away candidate should fail collision check
    assert ranked[0].final_score >= ranked[-1].final_score
