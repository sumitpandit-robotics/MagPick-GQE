"""
grasp_quality_engine.py

Top-level orchestrator for the MagPick Grasp Quality Evaluation Framework.

This module ties together gripper profile loading, billet-gripper
compatibility checking, candidate evaluation, ranking, and report
generation — the single entry point for a production deployment.

Usage
-----
    from magpick.grasp_quality_engine import GraspQualityEngine

    engine = GraspQualityEngine(
        gripper_profile="config/grippers/schmalz_sgm_hp_40x121.yaml",
    )
    report = engine.evaluate(
        candidates=candidate_poses,
        billet=billet,
        scene=scene,
    )
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from magpick.config import config
from magpick.gripper_profile import GripperProfile
from magpick.models import (
    Billet,
    CandidatePose,
    CandidateResult,
    Gripper,
    RobotMotion,
    Scene,
)
from magpick.fusion import FusionEngine


# ==========================================================
# Compatibility Pre-Check
# ==========================================================

@dataclass
class CompatibilityResult:
    """Result of a billet-gripper compatibility check."""

    compatible: bool
    message: str
    details: Dict = field(default_factory=dict)


def check_gripper_billet_compatibility(
    gripper: Gripper,
    billet: Billet,
    robot_motion: Optional[RobotMotion] = None,
    config_override: Optional[Dict] = None,
) -> CompatibilityResult:
    """Verify that the gripper can physically hold the billet.

    This runs the best-case magnetic holding-force calculation
    (material_factor × surface_factor × rated_force, with
    curvature-induced air gap) against the required force
    (gravity + dynamic loads × safety factor).  If even the
    optimistic case fails, the evaluation should not proceed.

    Parameters
    ----------
    gripper : Gripper
        The gripper to evaluate.
    billet : Billet
        The billet to pick.
    robot_motion : RobotMotion, optional
        Robot dynamic parameters.  If None, only gravity is considered.
    config_override : dict, optional
        Override the default magnetic config (for testing).

    Returns
    -------
    CompatibilityResult
    """
    if config_override is not None:
        magnetic_cfg = config_override.get("magnetic", config["magnetic"])
    else:
        magnetic_cfg = config["magnetic"]

    # --- Best-case material / surface factors ---
    mat_lookup = magnetic_cfg.get("material_factor", {})
    mat_factor = mat_lookup.get(billet.material)
    mat_recognized = mat_factor is not None
    if mat_factor is None:
        mat_factor = 0.0  # fail-safe: worst case

    surf_lookup = magnetic_cfg.get("surface_factor", {})
    surf_factor = surf_lookup.get(billet.surface)
    if surf_factor is None:
        surf_factor = 0.0

    # --- Best-case holding force (zero additional air gap) ---
    import math
    curvature_gap_m = 0.0
    if gripper.pad_width > 0 and billet.radius > gripper.pad_width / 2:
        half_chord = gripper.pad_width / 2.0
        curvature_gap_m = billet.radius - math.sqrt(
            billet.radius ** 2 - half_chord ** 2
        )
    curvature_gap_mm = curvature_gap_m * 1000.0

    # Air gap factor at curvature gap only (no user-specified additional gap)
    air_gap_derating = 1.0
    if curvature_gap_mm > 0:
        # Inline the same staircase as MagneticEvaluator.air_gap_factor
        if curvature_gap_mm <= 0.2:
            air_gap_derating = 0.95
        elif curvature_gap_mm <= 0.5:
            air_gap_derating = 0.85
        elif curvature_gap_mm <= 1.0:
            air_gap_derating = 0.70
        elif curvature_gap_mm <= 2.0:
            air_gap_derating = 0.50
        else:
            air_gap_derating = 0.25

    best_case_holding = (
        gripper.max_force * mat_factor * surf_factor * air_gap_derating
    )

    # --- Required force ---
    gravity = billet.weight * 9.81
    dynamic = 0.0
    if robot_motion is not None:
        dynamic = billet.weight * robot_motion.acceleration
    required = gravity + dynamic

    # --- Safety factor ---
    safety_factor = best_case_holding / required if required > 0 else 0.0
    min_sf = magnetic_cfg.get("minimum_safety_factor", 2.0)

    details = {
        "best_case_holding_force_N": best_case_holding,
        "required_force_N": required,
        "best_case_safety_factor": safety_factor,
        "minimum_safety_factor": min_sf,
        "material_factor": mat_factor,
        "material_recognized": mat_recognized,
        "surface_factor": surf_factor,
        "curvature_gap_mm": curvature_gap_mm,
        "air_gap_derating": air_gap_derating,
    }

    if not mat_recognized:
        return CompatibilityResult(
            compatible=False,
            message=(
                f"Unrecognised material '{billet.material}'.  "
                f"Add it to the material_factor table in config/weights.yaml "
                f"or set billet.material to a known value."
            ),
            details=details,
        )

    if safety_factor < min_sf:
        return CompatibilityResult(
            compatible=False,
            message=(
                f"Billet {billet.radius*2*1000:.0f}mm dia exceeds "
                f"{gripper.name} capacity: "
                f"best-case holding force {best_case_holding:.0f}N "
                f"< required {required:.0f}N "
                f"(safety factor {safety_factor:.2f} < {min_sf:.2f}).  "
                f"Select a higher-capacity gripper or verify billet "
                f"weight/material."
            ),
            details=details,
        )

    return CompatibilityResult(
        compatible=True,
        message="Gripper-billet compatibility OK.",
        details=details,
    )


# ==========================================================
# Billet Type Library
# ==========================================================

def _load_billet_types(config_dir: str = "config") -> List[Dict]:
    """Load the billet type library from config/billet_types.yaml."""
    from pathlib import Path
    import yaml

    path = Path(config_dir) / "billet_types.yaml"
    if not path.exists():
        return []
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data or []


def match_billet_type(
    billet: Billet,
    billet_types: Optional[List[Dict]] = None,
    tolerance_mm: float = 2.0,
) -> Optional[Dict]:
    """Match a billet against the known billet type library.

    Parameters
    ----------
    billet : Billet
        The measured billet (dimensions in metres).
    billet_types : list of dict, optional
        The billet type library.  If None, loaded from config.
    tolerance_mm : float
        Maximum allowed deviation from nominal diameter/length (mm).

    Returns
    -------
    dict or None
        The matched billet type entry, or None if no match.
    """
    if billet_types is None:
        billet_types = _load_billet_types()

    dia_mm = billet.radius * 2 * 1000.0
    len_mm = billet.length * 1000.0

    for bt in billet_types:
        nom_dia = bt.get("nominal_diameter_mm", 0)
        nom_len = bt.get("nominal_length_mm", 0)
        tol = bt.get("tolerance_mm", tolerance_mm)

        dia_ok = abs(dia_mm - nom_dia) <= tol
        len_ok = abs(len_mm - nom_len) <= tol

        if dia_ok and len_ok:
            return bt

    return None


# ==========================================================
# Report Data
# ==========================================================

@dataclass
class EvaluationReport:
    """Complete evaluation report for a set of candidates."""

    candidates: List[CandidateResult]
    billet: Billet
    gripper: Gripper
    compatibility: CompatibilityResult
    best: Optional[CandidateResult] = None
    summary: Dict = field(default_factory=dict)

    def __post_init__(self):
        if self.best is None and self.candidates:
            passed = [c for c in self.candidates if c.final_score > 0]
            if passed:
                self.best = max(passed, key=lambda c: c.final_score)

        total = len(self.candidates)
        passed_count = sum(1 for c in self.candidates if c.final_score > 0)
        self.summary = {
            "total_candidates": total,
            "passed": passed_count,
            "failed": total - passed_count,
            "best_score": self.best.final_score if self.best else 0.0,
            "compatible": self.compatibility.compatible,
        }


# ==========================================================
# Grasp Quality Engine
# ==========================================================

class GraspQualityEngine:
    """Top-level orchestrator for grasp quality evaluation.

    Parameters
    ----------
    gripper_profile : str or GripperProfile
        Path to a gripper YAML profile, or a pre-loaded profile.
    """

    def __init__(self, gripper_profile):
        if isinstance(gripper_profile, (str,)):
            self.profile = GripperProfile.load(gripper_profile)
        else:
            self.profile = gripper_profile

        self.gripper = Gripper.from_profile(self.profile)
        self.fusion = FusionEngine()

    def check_compatibility(
        self,
        billet: Billet,
        robot_motion: Optional[RobotMotion] = None,
    ) -> CompatibilityResult:
        """Pre-flight check: can this gripper hold this billet?"""
        return check_gripper_billet_compatibility(
            self.gripper, billet, robot_motion,
        )

    def evaluate(
        self,
        candidates: List[CandidatePose],
        billet: Billet,
        scene: Scene,
        robot_motion: Optional[RobotMotion] = None,
        check_compatibility: bool = True,
    ) -> EvaluationReport:
        """Evaluate all candidates and return a ranked report.

        Parameters
        ----------
        candidates : list of CandidatePose
            Grasp candidates from the planner.
        billet : Billet
            The billet to pick.
        scene : Scene
            The perception scene (point cloud + metadata).
        robot_motion : RobotMotion, optional
            Robot dynamic parameters for force calculation.
        check_compatibility : bool
            Run the pre-flight compatibility check.  Default True.

        Returns
        -------
        EvaluationReport
        """
        # --- Pre-flight compatibility check ---
        compat = self.check_compatibility(billet, robot_motion)
        if check_compatibility and not compat.compatible:
            # Still return a report, but all candidates are failed
            empty_results = [
                CandidateResult(
                    candidate=c,
                    final_score=0.0,
                    status="INCOMPATIBLE",
                )
                for c in candidates
            ]
            return EvaluationReport(
                candidates=empty_results,
                billet=billet,
                gripper=self.gripper,
                compatibility=compat,
            )

        # --- Run FusionEngine evaluation ---
        ranked = self.fusion.rank_candidates(
            candidates, billet, self.gripper, scene, robot_motion,
        )

        return EvaluationReport(
            candidates=ranked,
            billet=billet,
            gripper=self.gripper,
            compatibility=compat,
        )
