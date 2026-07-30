"""
contact_area.py

Contact Area Evaluator

Estimates the effective magnetic contact area between the
Schmalz magnetic gripper and a cylindrical billet.
"""

import numpy as np

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator
from magpick.config import config


class ContactAreaEvaluator(BaseEvaluator):

    def __init__(self):
        self.cfg = config["contact"]

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
            weight=self.cfg["weight"],
            reason="Contact area evaluated.",
            details=metrics,
        )

    def compute_metrics(self, candidate, billet, gripper):

        billet_diameter = billet.radius * 2.0

        width_coverage = min(billet_diameter / gripper.pad_width, 1.0)
        length_coverage = min(billet.length / gripper.pad_length, 1.0)

        # Limiting axis determines actual coverage — a billet shorter than the
        # pad's length can't achieve full contact even if its diameter matches
        # the pad width, and vice versa.
        coverage_ratio = min(width_coverage, length_coverage)

        return {
            "billet_diameter": billet_diameter,
            "billet_length": billet.length,
            "pad_width": gripper.pad_width,
            "pad_length": gripper.pad_length,
            "width_coverage": width_coverage,
            "length_coverage": length_coverage,
            "coverage_ratio": coverage_ratio,
            "contact_factor": coverage_ratio,
        }

    def compute_score(
        self,
        metrics,
    ):

        return metrics["contact_factor"]