"""
contact_area.py

Contact Area Evaluator

Estimates the effective magnetic contact area between the
Schmalz magnetic gripper and a cylindrical billet.
"""

import numpy as np

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator


class ContactAreaEvaluator(BaseEvaluator):

    def evaluate(
        self,
        candidate,
        billet,
        gripper,
        scene,
        robot_motion=None,
    ) -> EvaluationResult:

        metrics = self.compute_metrics(
            candidate,
            billet,
            gripper,
        )

        score = self.compute_score(metrics)

        return EvaluationResult(
            name="Contact Area",
            passed=True,
            score=score,
            weight=1.0,
            reason="Contact area evaluated.",
            details=metrics,
        )

    def compute_metrics(
        self,
        candidate,
        billet,
        gripper,
    ):

        billet_diameter = billet.radius * 2.0

        gripper_width = gripper.pad_diameter

        coverage_ratio = min(
            billet_diameter / gripper_width,
            1.0,
        )

        contact_factor = coverage_ratio

        return {

            "billet_diameter": billet_diameter,

            "gripper_width": gripper_width,

            "coverage_ratio": coverage_ratio,

            "contact_factor": contact_factor,
        }

    def compute_score(
        self,
        metrics,
    ):

        return metrics["contact_factor"]