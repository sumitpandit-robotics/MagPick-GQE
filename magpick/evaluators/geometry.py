"""
geometry.py

Geometry Evaluator

Evaluates purely geometric grasp quality.

Author: Sumit Pandit
"""

from magpick.models import (
    CandidatePose,
    Billet,
    Gripper,
    Scene,
    EvaluationResult,
)

from magpick.evaluators.base import BaseEvaluator
from magpick.config import config


class GeometryEvaluator(BaseEvaluator):
    """
    Evaluates geometric quality of a grasp candidate.
    """

    def __init__(self):

        self.cfg = config["geometry"]

    # ==========================================================
    # Main Evaluation
    # ==========================================================

    def evaluate(
        self,
        candidate: CandidatePose,
        billet: Billet,
        gripper: Gripper,
        scene: Scene,
        robot_motion=None,
    ) -> EvaluationResult:

        metrics = self.compute_metrics(
            candidate,
            billet,
            gripper,
            scene,
        )

        score = self.compute_score(metrics)

        passed = score >= self.cfg["minimum_score"]

        return EvaluationResult(
            name="Geometry",
            passed=passed,
            score=score,
            weight=self.cfg["weight"],
            reason="Geometry evaluation completed.",
            details=metrics,
        )

    # ==========================================================
    # Compute Metrics
    # ==========================================================

    def compute_metrics(
        self,
        candidate: CandidatePose,
        billet: Billet,
        gripper: Gripper,
        scene: Scene,
    ):

        from magpick.utils.geometry import (
            nearest_point,
            compute_normal,
            quaternion_to_approach,
            angle_between,
        )

        # ------------------------------------------------------
        # Point cloud
        # ------------------------------------------------------

        pcd = scene.point_cloud

        # ------------------------------------------------------
        # Find nearest point
        # ------------------------------------------------------

        idx = nearest_point(
            pcd,
            candidate.position,
        )

        surface_normal = compute_normal(
            pcd,
            idx,
        )

        # ------------------------------------------------------
        # Compute approach vector
        # ------------------------------------------------------

        approach = quaternion_to_approach(
            candidate.orientation,
        )

        # Store for debugging / later evaluators

        candidate.approach_vector = approach
        candidate.surface_normal = surface_normal

        # ------------------------------------------------------
        # Geometry Metric
        # ------------------------------------------------------

        normal_error = angle_between(
            approach,
            surface_normal,
        )

        return {

            "normal_error_deg": float(normal_error),

            # Placeholder metrics (implemented later)

            "contact_ratio": 1.0,

            "overhang_ratio": 0.0,

            "curvature_score": 1.0,

        }

    # ==========================================================
    # Compute Score
    # ==========================================================

    def compute_score(self, metrics):

        score = 1.0

        error = metrics["normal_error_deg"]

        max_error = self.cfg["max_normal_error_deg"]

        if error > max_error:

            penalty = (error - max_error) / 90.0

            score *= max(0.0, 1.0 - penalty)

        score *= metrics["contact_ratio"]

        score *= (1.0 - metrics["overhang_ratio"])

        score *= metrics["curvature_score"]

        return max(0.0, min(score, 1.0))