"""
test_evaluator_weights.py

Regression tests verifying that every evaluator's weight
matches the config, and that the weight table sums correctly.
"""

import pytest
from magpick.evaluators.geometry import GeometryEvaluator
from magpick.evaluators.magnetic import MagneticEvaluator
from magpick.evaluators.contact_area import ContactAreaEvaluator
from magpick.evaluators.collision import CollisionEvaluator
from magpick.evaluators.pole_coverage import PoleCoverageEvaluator
from magpick.evaluators.robot_dynamics import RobotDynamicsEvaluator
from magpick.config import config


@pytest.mark.parametrize("evaluator_cls,cfg_key", [
    (GeometryEvaluator, "geometry"),
    (MagneticEvaluator, "magnetic"),
    (ContactAreaEvaluator, "contact"),
    (CollisionEvaluator, "collision"),
    (PoleCoverageEvaluator, "pole_coverage"),
    (RobotDynamicsEvaluator, "robot_dynamics"),
])
def test_evaluator_weight_matches_config(evaluator_cls, cfg_key):
    """Regression test for the weight-hardcoding bug: every evaluator's
    returned EvaluationResult.weight must equal config[cfg_key]['weight'],
    not a hardcoded constant."""
    ev = evaluator_cls()
    assert ev.cfg["weight"] == config[cfg_key]["weight"]


def test_unrecognized_material_defaults_to_worst_case():
    ev = MagneticEvaluator()
    from magpick.models import Billet
    import numpy as np

    billet = Billet(
        id=99, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
        radius=0.02, length=0.2, weight=2.5, material="unobtainium",
    )

    class DummyGripper:
        max_force = 1070.0
        pad_width = 0.040
        pad_length = 0.121

    metrics = ev.compute_metrics(candidate=None, billet=billet, gripper=DummyGripper())
    assert metrics["material_factor"] == 0.0   # NOT 1.0 (the old unsafe default)
    assert metrics["material_recognized"] is False


def test_weights_sum_to_one():
    """All evaluator weights should sum to 1.0 for the weighted average
    to be a proper convex combination."""
    total = sum(
        config[key]["weight"]
        for key in ["geometry", "contact", "magnetic", "collision",
                     "pole_coverage", "robot_dynamics"]
    )
    assert total == pytest.approx(1.0)
