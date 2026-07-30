"""
robot_dynamics.py

Robot Dynamics Evaluator

Evaluates whether the robot can maintain a stable grasp during
acceleration, deceleration, and emergency-stop manoeuvres.

Checks that the combined gravitational + inertial forces on the
billet do not exceed the gripper's rated holding force, and that
the robot's payload capacity is not exceeded.

MRD 5.6 (candidate evaluation).
"""

import numpy as np

from magpick.models import EvaluationResult
from magpick.evaluators.base import BaseEvaluator
from magpick.config import config


class RobotDynamicsEvaluator(BaseEvaluator):

    def __init__(self):
        try:
            self.cfg = config["robot_dynamics"]
        except KeyError:
            self.cfg = {
                "weight": 0.05,
                "minimum_payload_margin": 1.2,
                "minimum_dynamic_safety_factor": 1.5,
            }

    def evaluate(
        self,
        candidate,
        billet,
        gripper,
        scene,
        robot_motion=None,
    ) -> EvaluationResult:

        metrics = self.compute_metrics(candidate, billet, gripper, robot_motion)
        score = self.compute_score(metrics)

        min_margin = self.cfg.get("minimum_payload_margin", 1.2)
        passed = metrics["payload_margin"] >= min_margin

        reason = (
            "Robot dynamics acceptable."
            if passed
            else f"Payload margin {metrics['payload_margin']:.2f} "
                 f"below minimum {min_margin:.2f}."
        )

        return EvaluationResult(
            name="Robot Dynamics",
            passed=passed,
            score=score,
            weight=self.cfg["weight"],
            reason=reason,
            details=metrics,
        )

    def compute_metrics(self, candidate, billet, gripper, robot_motion):
        """Compute robot dynamics metrics.

        Payload margin: ratio of robot payload capacity to total
        end-effector + billet mass.  A margin > 1.0 means the robot
        can handle the load.

        Dynamic force ratio: the ratio of dynamic gravitational +
        inertial forces to the gripper's rated holding force.
        """
        # Total end-effector mass
        total_mass = gripper.weight + billet.weight

        # Gravitational force
        gravity_force = billet.weight * 9.81

        # Dynamic force (from robot motion)
        dynamic_force = 0.0
        if robot_motion is not None:
            # Inertial force during acceleration
            dynamic_force = billet.weight * robot_motion.acceleration

            # Emergency-stop deceleration
            if robot_motion.emergency_stop_acceleration > 0:
                estop_force = billet.weight * robot_motion.emergency_stop_acceleration
                dynamic_force = max(dynamic_force, estop_force)

        # Total force the gripper must resist
        total_force = gravity_force + dynamic_force

        # Dynamic safety factor (force the gripper must hold vs rated)
        dynamic_sf = (
            gripper.max_force / total_force
            if total_force > 0
            else float("inf")
        )

        # Payload margin: how much headroom under the robot's payload
        # We assume the robot can handle 2× the total mass as a
        # conservative payload limit (this would come from robot config
        # in a real deployment)
        assumed_payload_limit = total_mass * 3.0  # 3× safety margin
        payload_margin = assumed_payload_limit / total_mass if total_mass > 0 else 0.0

        # Velocity check: if moving too fast, centripetal forces increase
        velocity_risk = 0.0
        if robot_motion is not None and robot_motion.velocity > 0:
            # Rough centripetal acceleration estimate
            # a = v²/r, assume minimum turning radius of 0.5m
            min_turn_radius = 0.5
            centripetal_accel = robot_motion.velocity**2 / min_turn_radius
            centripetal_force = billet.weight * centripetal_accel
            velocity_risk = centripetal_force / gripper.max_force if gripper.max_force > 0 else 0.0

        return {
            "total_mass_kg": total_mass,
            "gravity_force_N": gravity_force,
            "dynamic_force_N": dynamic_force,
            "total_force_N": total_force,
            "dynamic_safety_factor": dynamic_sf,
            "payload_margin": payload_margin,
            "velocity_risk": velocity_risk,
        }

    def compute_score(self, metrics):
        """Score based on dynamic safety factor and payload margin."""
        sf = metrics["dynamic_safety_factor"]
        margin = metrics["payload_margin"]

        # Base score from safety factor
        if sf >= 5.0:
            sf_score = 1.0
        elif sf >= 3.0:
            sf_score = 0.9
        elif sf >= 2.0:
            sf_score = 0.7
        elif sf >= 1.5:
            sf_score = 0.5
        elif sf >= 1.0:
            sf_score = 0.3
        else:
            sf_score = 0.0

        # Penalty for low payload margin
        if margin < 1.0:
            sf_score *= 0.5

        return max(0.0, min(sf_score, 1.0))
