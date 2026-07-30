"""
magnetic.py

Magnetic Grasp Evaluator

Evaluates whether a magnetic gripper can safely hold a ferromagnetic billet,
accounting for material permeability, surface condition, air gap (curvature-
induced and user-specified), and robot dynamic loads.
"""

import math

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator
from magpick.config import config


class MagneticEvaluator(BaseEvaluator):

    def __init__(self):
        self.cfg = config["magnetic"]

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
            robot_motion,
        )

        score = self.compute_score(metrics)

        passed = metrics["safety_factor"] >= self.cfg["minimum_safety_factor"]

        reason = (
            "Magnetic grasp accepted."
            if passed
            else f"Safety factor {metrics['safety_factor']:.2f} below "
                 f"minimum {self.cfg['minimum_safety_factor']:.2f}."
        )

        return EvaluationResult(
            name="Magnetic",
            passed=passed,
            score=score,
            weight=self.cfg["weight"],
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

        material_factor = self.cfg["material_factor"].get(billet.material)
        material_recognized = material_factor is not None
        if material_factor is None:
            # Fail-safe: an unrecognized material must NOT default to best
            # case (1.0, as good as forged steel). Assume worst case (0.0)
            # instead, and flag it explicitly via material_recognized.
            material_factor = 0.0

        # ------------------------------------------------------
        # Surface factor
        # ------------------------------------------------------

        surface_factor = self.cfg["surface_factor"].get(billet.surface)
        surface_recognized = surface_factor is not None
        if surface_factor is None:
            surface_factor = 0.0

        # ------------------------------------------------------
        # Air gap (curvature-induced + user-specified)
        # ------------------------------------------------------

        # For a round billet against a flat rectangular pad, the effective
        # standoff is the sagitta of the arc: r - sqrt(r^2 - (w/2)^2)
        # where w is the pad width (short side, the constraining dimension).
        curvature_gap_m = 0.0
        if gripper.pad_width > 0 and billet.radius > gripper.pad_width / 2:
            half_chord = gripper.pad_width / 2.0
            curvature_gap_m = billet.radius - math.sqrt(
                billet.radius ** 2 - half_chord ** 2
            )

        # billet.air_gap is any additional user-specified gap (coating,
        # paint, thermal expansion clearance) in mm.
        total_air_gap_mm = curvature_gap_m * 1000.0 + billet.air_gap

        air_gap_derating = self.air_gap_factor(total_air_gap_mm)

        # ------------------------------------------------------
        # Effective holding force
        # ------------------------------------------------------

        holding_force = (
            gripper.max_force
            * material_factor
            * surface_factor
            * air_gap_derating
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

        safety_factor = (
            holding_force / required_force
            if required_force > 0
            else 0.0
        )

        return {
            "holding_force": holding_force,
            "required_force": required_force,
            "safety_factor": safety_factor,
            "material_factor": material_factor,
            "surface_factor": surface_factor,
            "air_gap_derating": air_gap_derating,
            "total_air_gap_mm": total_air_gap_mm,
            "curvature_gap_mm": curvature_gap_m * 1000.0,
            "material_recognized": material_recognized,
            "surface_recognized": surface_recognized,
        }

    def compute_score(
        self,
        metrics,
    ):

        sf = metrics["safety_factor"]

        thresholds = self.cfg.get("score_thresholds", [])

        for entry in thresholds:
            if sf >= entry["min_safety_factor"]:
                return entry["score"]

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