#!/usr/bin/env python3
"""
MagPick-GQE End-to-End Test Script
Run:  PYTHONPATH="" .venv/bin/python run_test.py
"""

import numpy as np
import open3d as o3d
import sys
import os

PASS = 0
FAIL = 0

def check(label, condition):
    global PASS, FAIL
    if condition:
        PASS += 1
        print(f"  PASS  {label}")
    else:
        FAIL += 1
        print(f"  FAIL  {label}")


print("=" * 60)
print("MagPick-GQE — End-to-End Verification")
print("=" * 60)

# ----------------------------------------------------------
# 1. Config loading
# ----------------------------------------------------------
print("\n[1] Config Loading")
from magpick.config import config
check("weights.yaml loaded", "geometry" in config.data)
check("magnetic material_factor present", "material_factor" in config["magnetic"])
check("magnetic surface_factor present", "surface_factor" in config["magnetic"])
check("pole_coverage section present", "pole_coverage" in config.data)
check("robot_dynamics section present", "robot_dynamics" in config.data)

weights = ["geometry", "contact", "magnetic", "collision", "pole_coverage", "robot_dynamics"]
total_w = sum(config[k]["weight"] for k in weights)
check(f"Weights sum to 1.0 (got {total_w:.2f})", abs(total_w - 1.0) < 0.001)

# ----------------------------------------------------------
# 2. Gripper profile loading
# ----------------------------------------------------------
print("\n[2] Gripper Profile")
from magpick.gripper_profile import GripperProfile
from magpick.models import Gripper

profile = GripperProfile.load("config/grippers/schmalz_sgm_hp_40x121.yaml")
check(f"Profile name: {profile.name}", profile.name == "Schmalz SGM-HP 40x121")
check(f"Rated force: {profile.rated_force_n}N", profile.rated_force_n == 1070.0)
check(f"Footprint: {profile.footprint.width_m}x{profile.footprint.length_m}m",
      profile.footprint.width_m == 0.040 and profile.footprint.length_m == 0.121)
check(f"Force curve has {len(profile.force_curve)} entries", len(profile.force_curve) == 6)
check(f"Pole layout: {profile.pole_layout.num_poles} poles",
      profile.pole_layout.num_poles == 8)
check(f"COG: {profile.cog_m}", profile.cog_m is not None)

gripper = Gripper.from_profile(profile)
check(f"Gripper pad: {gripper.pad_width*1000:.0f}x{gripper.pad_length*1000:.0f}mm",
      gripper.pad_width == 0.040 and gripper.pad_length == 0.121)
check("Pole layout attached", gripper.pole_layout is not None)

# ----------------------------------------------------------
# 3. Billet model
# ----------------------------------------------------------
print("\n[3] Billet Model")
from magpick.models import Billet

billet = Billet(
    id=1, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
    radius=0.020, length=0.200, weight=2.5, material="forged_steel",
)
check(f"Billet radius: {billet.radius}m", billet.radius == 0.020)
check(f"Billet length: {billet.length}m", billet.length == 0.200)

billet_mm = Billet.from_mm(
    id=2, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
    radius_mm=20, length_mm=200, weight_kg=2.5,
)
check("Billet.from_mm() converts correctly", billet_mm.radius == 0.020)

try:
    bad = Billet(id=3, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
                 radius=50.0, length=0.2, weight=2.5)
    check("Billet rejects bad radius", False)
except ValueError:
    check("Billet rejects bad radius", True)

# ----------------------------------------------------------
# 4. Billet type matching
# ----------------------------------------------------------
print("\n[4] Billet Type Recognition")
from magpick.grasp_quality_engine import match_billet_type

match = match_billet_type(billet)
check(f"Matched SKU: {match['sku'] if match else 'None'}",
      match is not None and match["sku"] == "billet_40x200_steel")

no_match = Billet(id=99, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
                  radius=0.035, length=0.180, weight=3.0)
check("Unmatched billet returns None", match_billet_type(no_match) is None)

# ----------------------------------------------------------
# 5. Compatibility pre-check
# ----------------------------------------------------------
print("\n[5] Compatibility Pre-Check")
from magpick.grasp_quality_engine import check_gripper_billet_compatibility

compat = check_gripper_billet_compatibility(gripper, billet)
check(f"Steel billet compatible: {compat.compatible}", compat.compatible)
check(f"SF: {compat.details['best_case_safety_factor']:.2f}",
      compat.details["best_case_safety_factor"] > 2.0)

aluminium = Billet(id=99, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
                   radius=0.020, length=0.200, weight=2.5, material="aluminium")
compat_al = check_gripper_billet_compatibility(gripper, aluminium)
check("Aluminium rejected", not compat_al.compatible)
check("Aluminium factor=0.0", compat_al.details["material_factor"] == 0.0)

heavy = Billet(id=99, position=np.zeros(3), orientation=np.array([0, 0, 0, 1.0]),
               radius=0.020, length=0.200, weight=200.0)
compat_heavy = check_gripper_billet_compatibility(gripper, heavy)
check("200kg billet rejected", not compat_heavy.compatible)

# ----------------------------------------------------------
# 6. Individual evaluators
# ----------------------------------------------------------
print("\n[6] Evaluators")
from magpick.evaluators.geometry import GeometryEvaluator
from magpick.evaluators.magnetic import MagneticEvaluator
from magpick.evaluators.contact_area import ContactAreaEvaluator
from magpick.evaluators.collision import CollisionEvaluator
from magpick.evaluators.pole_coverage import PoleCoverageEvaluator
from magpick.evaluators.robot_dynamics import RobotDynamicsEvaluator
from magpick.models import CandidatePose, Scene, RobotMotion

pcd = o3d.io.read_point_cloud("datasets/scene.ply")
scene = Scene(point_cloud=pcd, frame_id="world")

candidate = CandidatePose(
    position=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
)

for EvaluatorCls, name in [
    (GeometryEvaluator, "Geometry"),
    (MagneticEvaluator, "Magnetic"),
    (ContactAreaEvaluator, "Contact Area"),
    (CollisionEvaluator, "Collision"),
    (PoleCoverageEvaluator, "Pole Coverage"),
    (RobotDynamicsEvaluator, "Robot Dynamics"),
]:
    ev = EvaluatorCls()
    result = ev.evaluate(candidate, billet, gripper, scene)
    check(f"{name}: score={result.score:.3f}, weight={result.weight:.2f}, passed={result.passed}",
          0.0 <= result.score <= 1.0 and result.weight > 0)

# ----------------------------------------------------------
# 7. FusionEngine ranking
# ----------------------------------------------------------
print("\n[7] FusionEngine")
from magpick.fusion import FusionEngine

engine = FusionEngine()
candidates = [
    CandidatePose(position=np.array([0.0, 0.0, 0.0]), orientation=np.array([0, 0, 0, 1.0])),
    CandidatePose(position=np.array([0.02, 0.0, 0.0]), orientation=np.array([0, 0, 0, 1.0])),
]
ranked = engine.rank_candidates(candidates, billet, gripper, scene)
check(f"Ranked {len(ranked)} candidates", len(ranked) == 2)
check("Ranking sorted descending", ranked[0].final_score >= ranked[1].final_score)
check("All scores in [0,1]", all(0 <= r.final_score <= 1 for r in ranked))
check("Has rank numbers", all(r.rank > 0 for r in ranked))
check("Has status", all(r.status in ("PASS", "FAIL") for r in ranked))
check("Evaluator results attached", all(len(r.evaluator_results) == 6 for r in ranked))

# ----------------------------------------------------------
# 8. Hard constraint enforcement
# ----------------------------------------------------------
print("\n[8] Hard Constraints")
bad_candidate = CandidatePose(
    position=np.array([999.0, 999.0, 999.0]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0]),
)
ranked2 = engine.rank_candidates([bad_candidate, candidates[0]], billet, gripper, scene)
check("Far-away candidate gets score=0", ranked2[-1].final_score == 0.0)
check("Near candidate score >= far candidate", ranked2[0].final_score >= ranked2[-1].final_score)

# ----------------------------------------------------------
# 9. GraspQualityEngine orchestrator
# ----------------------------------------------------------
print("\n[9] GraspQualityEngine")
from magpick.grasp_quality_engine import GraspQualityEngine

gqe = GraspQualityEngine("config/grippers/schmalz_sgm_hp_40x121.yaml")
report = gqe.evaluate(candidates, billet, scene)
check(f"Report has {len(report.candidates)} candidates", len(report.candidates) == 2)
check("Compatibility OK", report.compatibility.compatible)
check("Summary computed", report.summary["total_candidates"] == 2)
check("Best score is non-negative", report.summary["best_score"] >= 0)

incomp = gqe.evaluate(candidates, heavy, scene)
check("Incompatible billet -> all scores 0",
      all(c.final_score == 0 for c in incomp.candidates))

# ----------------------------------------------------------
# 10. Report generation
# ----------------------------------------------------------
print("\n[10] Report Generation")
from magpick.report import generate_report

os.makedirs("output", exist_ok=True)
paths = generate_report(
    report, output_dir="output",
    gripper_mesh_path="assets/schmalz/SGM-HP_40x121.obj",
    pole_layout=gqe.profile.pole_layout,
)
check("HTML report generated", "html" in paths and os.path.exists(paths["html"]))
check("JSON report generated", "json" in paths and os.path.exists(paths["json"]))
check("CSV report generated", "csv" in paths and os.path.exists(paths["csv"]))

# ----------------------------------------------------------
# Summary
# ----------------------------------------------------------
print("\n" + "=" * 60)
total = PASS + FAIL
print(f"RESULTS: {PASS}/{total} passed, {FAIL} failed")
print("=" * 60)

if FAIL > 0:
    sys.exit(1)
else:
    print("\nAll checks passed. Framework is operational.")
    print("Open output/evaluation_report.html to see the HTML report.")
    sys.exit(0)
