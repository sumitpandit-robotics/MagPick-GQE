"""
callbacks.py

All Dash callbacks for the MagPick-GQE Dashboard.
"""

import json
import base64
import os
import tempfile
import numpy as np
from dash import Input, Output, State, callback_context, no_update, html
from dash.exceptions import PreventUpdate
import dash_bootstrap_components as dbc

from magpick.ui.inputs import GRIPPER_PRESETS
from magpick.ui.viewport_3d import render_scene, CAMERA_PRESETS
from magpick.ui.results import render_candidate_list, render_candidate_detail
from magpick.ui.utils import downsample_pcd
from magpick.models import Billet, CandidatePose, Scene
from magpick.grasp_quality_engine import GraspQualityEngine
from magpick.report import generate_report


def register_callbacks(app):

    # ==========================================================
    # Load preset values into gripper fields
    # ==========================================================
    @app.callback(
        Output("gripper-name", "value"),
        Output("gripper-force", "value"),
        Output("gripper-pad-w", "value"),
        Output("gripper-pad-l", "value"),
        Output("gripper-weight", "value"),
        Output("gripper-tcp-depth", "value"),
        Output("gripper-num-poles", "value"),
        Output("gripper-poles", "value"),
        Output("gripper-force-curve", "value"),
        Output("gripper-cog", "value"),
        Input("gripper-preset", "value"),
    )
    def load_preset(preset_key):
        if not preset_key or preset_key == "custom":
            return (no_update,) * 10
        p = GRIPPER_PRESETS.get(preset_key, {})
        if not p:
            return (no_update,) * 10
        return (
            p.get("name", ""),
            p.get("rated_force_n", 0),
            p.get("pad_width_mm", 0),
            p.get("pad_length_mm", 0),
            p.get("weight_kg", 0),
            p.get("tcp_depth_mm", 0),
            p.get("num_poles", 0),
            p.get("pole_positions", "[]"),
            p.get("force_curve", "{}"),
            p.get("cog", "[0,0,0]"),
        )

    # ==========================================================
    # File upload display names
    # ==========================================================
    @app.callback(Output("gripper-mesh-name", "children"), Input("gripper-mesh-upload", "filename"))
    def show_mesh_name(filename):
        return filename or ""

    @app.callback(Output("scene-file-name", "children"), Input("scene-ply-upload", "filename"))
    def show_scene_name(filename):
        return filename or ""

    @app.callback(Output("poses-file-name", "children"), Input("poses-json-upload", "filename"))
    def show_poses_name(filename):
        return filename or ""

    # ==========================================================
    # 3D Camera presets
    # ==========================================================
    @app.callback(
        Output("scene-3d", "figure"),
        Input("cam-iso", "n_clicks"),
        Input("cam-top", "n_clicks"),
        Input("cam-front", "n_clicks"),
        Input("cam-side", "n_clicks"),
        State("scene-3d", "figure"),
        prevent_initial_call=True,
    )
    def set_camera(*args):
        ctx = callback_context
        if not ctx.triggered:
            raise PreventUpdate
        btn_id = ctx.triggered[0]["prop_id"].split(".")[0]
        preset_map = {"cam-iso": "iso", "cam-top": "top", "cam-front": "front", "cam-side": "side"}
        preset = CAMERA_PRESETS.get(preset_map.get(btn_id, "iso"))
        fig_data = args[-1]
        if fig_data and "layout" in fig_data and "scene" in fig_data["layout"]:
            fig_data["layout"]["scene"]["camera"]["eye"] = preset["eye"]
            fig_data["layout"]["scene"]["camera"]["up"] = preset["up"]
            return fig_data
        raise PreventUpdate

    # ==========================================================
    # Main: Run Evaluation
    # ==========================================================
    @app.callback(
        Output("scene-3d", "figure", allow_duplicate=True),
        Output("card-best-score", "children"),
        Output("card-compat", "children"),
        Output("card-total", "children"),
        Output("card-passed", "children"),
        Output("card-failed", "children"),
        Output("card-rate", "children"),
        Output("candidate-list-body", "children"),
        Output("candidate-detail", "children"),
        Output("comparison-chart", "figure"),
        Output("eval-store", "data"),
        Output("viewport-info", "children"),
        Input("run-btn", "n_clicks"),
        State("gripper-preset", "value"),
        State("gripper-name", "value"),
        State("gripper-force", "value"),
        State("gripper-pad-w", "value"),
        State("gripper-pad-l", "value"),
        State("gripper-weight", "value"),
        State("gripper-tcp-depth", "value"),
        State("gripper-num-poles", "value"),
        State("gripper-poles", "value"),
        State("gripper-force-curve", "value"),
        State("gripper-cog", "value"),
        State("billet-diameter", "value"),
        State("billet-length", "value"),
        State("billet-weight", "value"),
        State("billet-material", "value"),
        State("billet-surface", "value"),
        State("billet-air-gap", "value"),
        State("scene-use-example", "value"),
        State("resolution-slider", "value"),
        State("poses-manual", "value"),
        State("scene-ply-upload", "contents"),
        State("scene-ply-upload", "filename"),
        State("poses-json-upload", "contents"),
        prevent_initial_call=True,
    )
    def run_evaluation(n_clicks, preset, g_name, g_force, g_padw, g_padl, g_weight, g_tcp,
                       g_poles_n, g_poles, g_fc, g_cog, b_dia, b_len, b_wt, b_mat, b_surf, b_gap,
                       scene_opt, resolution, poses_manual, scene_contents, scene_filename,
                       poses_contents):
        if not n_clicks:
            raise PreventUpdate

        try:
            # ---- Load scene ----
            pcd = None
            if "example" in (scene_opt or []):
                import open3d as o3d
                pcd = o3d.io.read_point_cloud("datasets/scene.ply")
            elif scene_contents is not None:
                _, ext = os.path.splitext(scene_filename or "scene.ply")
                content_type, content_string = scene_contents.split(",")
                decoded = base64.b64decode(content_string)
                with tempfile.NamedTemporaryFile(suffix=ext, delete=False) as f:
                    f.write(decoded)
                    tmp_path = f.name
                import open3d as o3d
                pcd = o3d.io.read_point_cloud(tmp_path)
                os.unlink(tmp_path)

            # Downsample
            resolution_map = {0: 10000, 1: 50000, 2: 100000, 3: 99999999}
            target = resolution_map.get(resolution or 1, 50000)
            if pcd is not None:
                pcd = downsample_pcd(pcd, target)

            scene = Scene(point_cloud=pcd, frame_id="camera_optical_frame") if pcd is not None else Scene(point_cloud=None, frame_id="world")

            # ---- Load poses ----
            billet_poses = []
            if poses_contents is not None:
                _, ext = os.path.splitext("poses.json")
                content_type, content_string = poses_contents.split(",")
                decoded = base64.b64decode(content_string)
                data = json.loads(decoded.decode("utf-8"))
                billet_poses = data.get("billets", [])
            elif poses_manual and poses_manual.strip():
                data = json.loads(poses_manual)
                billet_poses = data.get("billets", [])

            if not billet_poses:
                empty_fig = render_scene(pcd, [], [], [])
                err = html.Div([html.H5("No poses loaded", className="text-danger"), html.P("Upload a .json file or enter poses manually")])
                return empty_fig, "--", "--", "0", "0", "0", "--", err, err, go.Figure(), {}, "No poses"

            # ---- Create billets & candidates ----
            billets = []
            candidates = []
            for bp in billet_poses:
                pos = np.array(bp["position"])
                orient = np.array(bp["orientation"])
                b = Billet.from_mm(
                    id=bp.get("id", len(billets)),
                    position=pos, orientation=orient,
                    radius_mm=(b_dia or 65) / 2,
                    length_mm=b_len or 175,
                    weight_kg=b_wt or 3.5,
                    material=b_mat or "forged_steel",
                    surface=b_surf or "clean",
                    air_gap_mm=b_gap or 0,
                )
                billets.append(b)
                # Generate grasp candidates: top-down + 2 angled
                tcp_depth = (g_tcp or 103.4) / 1000.0
                for angle_deg in [0, 15, -15]:
                    from scipy.spatial.transform import Rotation
                    if angle_deg == 0:
                        q = np.array([0.0, 0.0, 0.0, 1.0])
                    else:
                        q = Rotation.from_euler("y", angle_deg, degrees=True).as_quat()
                    cp = CandidatePose(
                        position=pos + np.array([0, 0, tcp_depth + 0.05]),
                        orientation=q,
                    )
                    candidates.append(cp)

            # ---- Run evaluation ----
            gqe = GraspQualityEngine("config/grippers/schmalz_sgm_hp_40x121.yaml")
            report = gqe.evaluate(candidates, billets[0], scene)

            scores = [r.final_score for r in report.candidates]
            status = "PASS" if report.compatibility.compatible else "FAIL"

            # ---- 3D viewport ----
            billet_dicts = [{"position": b.position.tolist(), "radius": b.radius, "length": b.length, "id": b.id} for b in billets]
            fig = render_scene(pcd, billet_dicts, candidates, scores)

            # ---- Summary cards ----
            best = f"{report.summary['best_score']:.3f}" if report.summary["best_score"] > 0 else "N/A"
            passed_n = report.summary["passed"]
            failed_n = report.summary["failed"]
            total_n = report.summary["total_candidates"]
            rate = f"{(passed_n / total_n * 100):.0f}%" if total_n > 0 else "0%"

            # ---- Candidate list ----
            cand_list = render_candidate_list(candidates, scores)

            # ---- First candidate detail ----
            first_detail = render_candidate_detail(
                report.candidates[0].candidate if report.candidates else None,
                report.candidates[0].evaluator_results if report.candidates else [],
                report.candidates[0].final_score if report.candidates else 0,
            )

            # ---- Comparison chart ----
            comp_fig = go.Figure(go.Bar(
                x=[f"#{i+1}" for i in range(len(scores))],
                y=scores,
                marker_color=["#27ae60" if s > 0 else "#e74c3c" for s in scores],
                text=[f"{s:.2f}" for s in scores],
                textposition="outside",
            ))
            comp_fig.update_layout(margin=dict(l=40, r=20, t=30, b=40), yaxis=dict(range=[0, 1.1]), title="Candidate Scores")

            # ---- Store evaluation data ----
            eval_data = {
                "candidates_positions": [c.position.tolist() for c in candidates],
                "candidates_orientations": [c.orientation.tolist() for c in candidates],
                "scores": scores,
                "evaluator_names": [ev.name for ev in report.candidates[0].evaluator_results] if report.candidates else [],
                "evaluator_scores": [[ev.score for ev in r.evaluator_results] for r in report.candidates],
                "evaluator_passed": [[ev.passed for ev in r.evaluator_results] for r in report.candidates],
                "evaluator_details": [{ev.name: ev.details for ev in r.evaluator_results} for r in report.candidates],
                "evaluator_reasons": [{ev.name: ev.reason for ev in r.evaluator_results} for r in report.candidates],
                "final_scores": scores,
                "billets": billet_dicts,
            }

            info = f"{total_n} candidates | {passed_n} passed | {failed_n} failed | Scene: {len(pcd.points) if pcd else 0} pts"

            return fig, best, status, str(total_n), str(passed_n), str(failed_n), rate, cand_list, first_detail, comp_fig, eval_data, info

        except Exception as e:
            import traceback
            err_fig = render_scene(None, [], [], [])
            err_div = html.Div([html.H5("Evaluation Error", className="text-danger"), html.Pre(str(e)), html.Pre(traceback.format_exc(), style={"fontSize": "10px"})])
            return err_fig, "ERROR", "ERROR", "0", "0", "0", "0%", err_div, err_div, go.Figure(), {}, str(e)

    # ==========================================================
    # Click candidate in list → show detail
    # ==========================================================
    @app.callback(
        Output("candidate-detail", "children", allow_duplicate=True),
        Input({"type": "candidate-row", "index": "__all__"}, "n_clicks"),
        State("eval-store", "data"),
        prevent_initial_call=True,
    )
    def on_candidate_click(n_clicks_list, eval_data):
        if not eval_data or not eval_data.get("scores"):
            raise PreventUpdate

        ctx = callback_context
        if not ctx.triggered:
            raise PreventUpdate

        # Determine which row was clicked from callback_context
        prop_id = ctx.triggered[0]["prop_id"]
        try:
            idx = json.loads(prop_id.split(".")[0])["index"]
        except (json.JSONDecodeError, KeyError):
            raise PreventUpdate

        if not isinstance(idx, int) or idx < 0:
            raise PreventUpdate

        scores = eval_data.get("final_scores", [])
        if idx >= len(scores):
            raise PreventUpdate

        candidates_pos = eval_data.get("candidates_positions", [])
        candidates_ori = eval_data.get("candidates_orientations", [])
        ev_names = eval_data.get("evaluator_names", [])
        ev_scores_all = eval_data.get("evaluator_scores", [])
        ev_passed_all = eval_data.get("evaluator_passed", [])
        ev_details_all = eval_data.get("evaluator_details", [])
        ev_reasons_all = eval_data.get("evaluator_reasons", [])

        # Rebuild CandidatePose
        from magpick.models import CandidatePose, EvaluationResult
        cp = CandidatePose(
            position=np.array(candidates_pos[idx]),
            orientation=np.array(candidates_ori[idx]),
        )
        # Rebuild evaluator results
        evaluator_results = []
        for j, name in enumerate(ev_names):
            details = ev_details_all[idx].get(name, {}) if idx < len(ev_details_all) else {}
            reasons = ev_reasons_all[idx].get(name, "") if idx < len(ev_reasons_all) else ""
            er = EvaluationResult(
                name=name,
                passed=ev_passed_all[idx][j] if idx < len(ev_passed_all) and j < len(ev_passed_all[idx]) else False,
                score=ev_scores_all[idx][j] if idx < len(ev_scores_all) and j < len(ev_scores_all[idx]) else 0,
                weight=0, reason=reasons, details=details,
            )
            evaluator_results.append(er)

        cp.rank = idx + 1
        detail = render_candidate_detail(cp, evaluator_results, scores[idx])
        return detail
