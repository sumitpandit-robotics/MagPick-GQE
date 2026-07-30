"""
collision.py

Collision Evaluator

Version 1

Evaluates the local clearance around a grasp candidate
using the scene point cloud.
"""

import numpy as np

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator


class CollisionEvaluator(BaseEvaluator):

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
            scene,
        )

        score = self.compute_score(metrics)

        passed = score > 0.5

        return EvaluationResult(
            name="Collision",
            passed=passed,
            score=score,
            weight=1.0,
            reason="Collision evaluated.",
            details=metrics,
        )

    def compute_metrics(
        self,
        candidate,
        scene,
    ):

        pcd = scene.point_cloud

        points = np.asarray(pcd.points)

        if len(points) == 0:

            return {
                "nearby_points": 0,
                "clearance_score": 1.0,
            }

        candidate_position = candidate.position

        distances = np.linalg.norm(
            points - candidate_position,
            axis=1,
        )

        radius = 0.05      # 50 mm

        nearby_points = np.sum(
            distances < radius
        )

        max_allowed = 500

        clearance_score = 1.0 - min(
            nearby_points / max_allowed,
            1.0,
        )

        return {

            "nearby_points": int(nearby_points),

            "search_radius": radius,

            "clearance_score": clearance_score,
        }

    def compute_score(
        self,
        metrics,
    ):

        return metrics["clearance_score"]