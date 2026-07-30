"""
magnetic.py

Magnetic Grasp Evaluator
"""

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator


class MagneticEvaluator(BaseEvaluator):

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
            robot_motion
        )

        score = self.compute_score(metrics)

        passed = metrics["safety_factor"] >= 1.5

        reason = (
            "Magnetic grasp accepted."
            if passed
            else "Insufficient magnetic holding force."
        )

        return EvaluationResult(
            name="Magnetic",
            passed=passed,
            score=score,
            weight=1.0,
            reason=reason,
            details=metrics,
        )

    def compute_metrics(
        self,
        candidate,
        billet,
        gripper,
        robot_motion=None,
    ):
        """
        Compute magnetic grasp metrics.
        """

        # ------------------------------------------------------
        # Material factor
        # ------------------------------------------------------

        material_lookup = {
            "forged_steel": 1.00,
            "cast_iron": 0.90,
            "stainless": 0.35,
            "aluminium": 0.00,
        }

        material_factor = material_lookup.get(
            billet.material,
            1.0,
        )

        # ------------------------------------------------------
        # Surface factor
        # ------------------------------------------------------

        surface_lookup = {
            "clean": 1.00,
            "oily": 0.80,
            "rusty": 0.75,
            "scale": 0.70,
        }

        surface_factor = surface_lookup.get(
            billet.surface,
            1.0,
        )

        # ------------------------------------------------------
        # Effective holding force
        # ------------------------------------------------------

        holding_force = (
            gripper.max_force
            * material_factor
            * surface_factor
        )

        # ------------------------------------------------------
        # Required holding force
        # ------------------------------------------------------

        gravity_force = billet.weight * 9.81

        dynamic_force = 0.0

        if robot_motion is not None:
            dynamic_force = (
                billet.weight
                * robot_motion.acceleration
            )
        required_force = (
            gravity_force
            + dynamic_force
        )

        # ------------------------------------------------------
        # Safety factor
        # ------------------------------------------------------

        safety_factor = holding_force / required_force

        return {
            "holding_force": holding_force,
            "required_force": required_force,
            "safety_factor": safety_factor,
            "material_factor": material_factor,
            "surface_factor": surface_factor,
        }

    def compute_score(
        self,
        metrics,
    ):

        sf = metrics["safety_factor"]

        if sf >= 3.0:
            return 1.0

        if sf >= 2.0:
            return 0.8

        if sf >= 1.5:
            return 0.6

        if sf >= 1.2:
            return 0.4

        return 0.0

    def air_gap_factor(
        self,
        air_gap_mm: float,
    ) -> float:
        """
        Estimate magnetic force reduction due to air gap.

        Returns a factor between 0 and 1.
        """

        if air_gap_mm <= 0.0:
            return 1.0

        if air_gap_mm <= 0.2:
            return 0.95

        if air_gap_mm <= 0.5:
            return 0.85

        if air_gap_mm <= 1.0:
            return 0.70

        if air_gap_mm <= 2.0:
            return 0.50

        return 0.25