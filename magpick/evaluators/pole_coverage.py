"""
pole_coverage.py

Pole Coverage Evaluator

Evaluates how well the magnetic poles of the gripper cover the
billet's contact surface.  For a rectangular gripper on a round
billet, the poles must be distributed across the contact area to
avoid uneven holding force and potential rotational instability.

MRD 5.2 (pole layout), MRD 5.6 (candidate evaluation).
"""

import numpy as np

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator
from magpick.config import config


class PoleCoverageEvaluator(BaseEvaluator):

    def __init__(self):
        try:
            self.cfg = config["pole_coverage"]
        except KeyError:
            self.cfg = {
                "weight": 0.05,
                "minimum_pole_coverage_ratio": 0.60,
            }

    def evaluate(
        self,
        candidate,
        billet,
        gripper,
        scene,
        robot_motion=None,
    ) -> EvaluationResult:

        metrics = self.compute_metrics(candidate, billet, gripper)
        score = self.compute_score(metrics)
        passed = score >= self.cfg.get("minimum_pole_coverage_ratio", 0.60)

        reason = (
            "Pole coverage adequate."
            if passed
            else f"Pole coverage ratio {metrics['pole_coverage_ratio']:.2f} "
                 f"below minimum {self.cfg.get('minimum_pole_coverage_ratio', 0.60):.2f}."
        )

        return EvaluationResult(
            name="Pole Coverage",
            passed=passed,
            score=score,
            weight=self.cfg["weight"],
            reason=reason,
            details=metrics,
        )

    def compute_metrics(self, candidate, billet, gripper):
        """Compute pole-to-billet coverage metrics.

        For each magnetic pole on the gripper, we check whether the
        pole falls within the contact zone on the billet surface.
        The contact zone is the rectangle defined by the pad footprint,
        projected onto the billet's cylindrical surface.

        Coverage ratio = (poles within contact zone) / (total poles)
        """
        # If gripper has no pole layout data, return a default
        pole_layout = getattr(gripper, "pole_layout", None)
        if pole_layout is None or pole_layout.num_poles == 0:
            # No pole data available — assume full coverage
            return {
                "num_poles": 0,
                "poles_in_contact": 0,
                "pole_coverage_ratio": 1.0,
                "billet_diameter_m": billet.radius * 2,
                "pad_width_m": gripper.pad_width,
                "pad_length_m": gripper.pad_length,
            }

        # Billet half-length in local frame (along pad long axis)
        billet_half_len = billet.length / 2.0
        # Billet radius
        r = billet.radius

        # Pad half-dimensions in local frame
        pad_half_w = gripper.pad_width / 2.0
        pad_half_l = gripper.pad_length / 2.0

        poles_in_contact = 0

        for pole_x, pole_y in pole_layout.pole_positions_m:
            # pole_x is along pad long axis (length), pole_y along short axis (width)
            # Check if the pole is within the pad footprint
            in_length = abs(pole_x) <= pad_half_l
            in_width = abs(pole_y) <= pad_half_w

            if not (in_length and in_width):
                continue

            # For a round billet, the pole must also be within the
            # billet's projected contact zone.  The contact zone is
            # limited by the billet's cylindrical curvature — a pole
            # near the edge of the pad may fall off the billet surface.
            # The billet contact width at position x along the length
            # is: w(x) = 2 * sqrt(r² - x²) for |x| <= r.
            # But the billet is along the pad's long axis, so we need
            # to check the pole's position along the pad's short axis
            # against the billet's circular cross-section.
            if abs(pole_x) <= r:
                max_contact_half_width = np.sqrt(r**2 - pole_x**2)
            else:
                max_contact_half_width = 0.0

            # The pole_y position must be within the contact zone
            if abs(pole_y) <= max_contact_half_width:
                poles_in_contact += 1

        total = pole_layout.num_poles
        coverage_ratio = poles_in_contact / total if total > 0 else 1.0

        return {
            "num_poles": total,
            "poles_in_contact": poles_in_contact,
            "pole_coverage_ratio": coverage_ratio,
            "billet_diameter_m": billet.radius * 2,
            "pad_width_m": gripper.pad_width,
            "pad_length_m": gripper.pad_length,
        }

    def compute_score(self, metrics):
        return metrics["pole_coverage_ratio"]
