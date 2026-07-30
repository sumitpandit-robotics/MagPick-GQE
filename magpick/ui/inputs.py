"""
inputs.py

Input panel components for the MagPick-GQE Dashboard.
"""

import dash_bootstrap_components as dbc
from dash import dcc, html

GRIPPER_PRESETS = {
    "schmalz_sgm_hp_40x121": {
        "name": "Schmalz SGM-HP 40x121",
        "rated_force_n": 1070.0,
        "pad_width_mm": 40,
        "pad_length_mm": 121,
        "weight_kg": 1.5,
        "tcp_depth_mm": 103.4,
        "num_poles": 8,
        "pole_positions": "[[-0.045,-0.010],[-0.015,-0.010],[0.015,-0.010],[0.045,-0.010],[-0.045,0.010],[-0.015,0.010],[0.015,0.010],[0.045,0.010]]",
        "force_curve": '{"0.0":1070,"0.5":850,"1.0":620,"2.0":340,"3.0":180,"5.0":60}',
        "cog": "[0.0, 0.0, -0.045]",
        "mesh_path": "assets/schmalz/SGM-HP_40x121.obj",
        "profile_path": "config/grippers/schmalz_sgm_hp_40x121.yaml",
    },
}


def make_gripper_section():
    return dbc.Accordion(
        [dbc.AccordionItem([
            html.Div([
                html.Label("Select Preset", className="form-label"),
                dcc.Dropdown(
                    id="gripper-preset",
                    options=[{"label": v["name"], "value": k} for k, v in GRIPPER_PRESETS.items()]
                    + [{"label": "Custom (fill below)", "value": "custom"}],
                    value="schmalz_sgm_hp_40x121",
                    className="mb-2",
                ),
            ]),
            html.Hr(),
            html.Div([
                html.Label("Upload YAML Profile", className="form-label"),
                dcc.Upload(
                    id="gripper-yaml-upload",
                    children=dbc.Button("Upload .yaml", color="secondary", size="sm", className="w-100"),
                    accept=".yaml,.yml",
                ),
            ], className="mb-2"),
            html.Div([
                html.Label("Upload Mesh (.obj/.stl)", className="form-label"),
                dcc.Upload(
                    id="gripper-mesh-upload",
                    children=dbc.Button("Upload mesh file", color="secondary", size="sm", className="w-100"),
                    accept=".obj,.stl,.ply",
                ),
                html.Div(id="gripper-mesh-name", className="text-muted small mt-1"),
            ], className="mb-2"),
            html.Hr(),
            dbc.Row([
                dbc.Col([html.Label("Name"), dcc.Input(id="gripper-name", type="text", value="Schmalz SGM-HP 40x121", className="form-control form-control-sm")], md=6),
                dbc.Col([html.Label("Rated Force (N)"), dcc.Input(id="gripper-force", type="number", value=1070, min=0, className="form-control form-control-sm")], md=6),
            ], className="mb-2"),
            dbc.Row([
                dbc.Col([html.Label("Pad W (mm)"), dcc.Input(id="gripper-pad-w", type="number", value=40, min=0, className="form-control form-control-sm")], md=4),
                dbc.Col([html.Label("Pad L (mm)"), dcc.Input(id="gripper-pad-l", type="number", value=121, min=0, className="form-control form-control-sm")], md=4),
                dbc.Col([html.Label("Weight (kg)"), dcc.Input(id="gripper-weight", type="number", value=1.5, min=0, step=0.1, className="form-control form-control-sm")], md=4),
            ], className="mb-2"),
            dbc.Row([
                dbc.Col([html.Label("TCP Depth (mm)"), dcc.Input(id="gripper-tcp-depth", type="number", value=103.4, min=0, className="form-control form-control-sm")], md=4),
                dbc.Col([html.Label("Num Poles"), dcc.Input(id="gripper-num-poles", type="number", value=8, min=0, className="form-control form-control-sm")], md=4),
                dbc.Col([html.Label("COG (x,y,z m)"), dcc.Input(id="gripper-cog", type="text", value="[0.0, 0.0, -0.045]", className="form-control form-control-sm")], md=4),
            ], className="mb-2"),
            html.Div([
                html.Label("Pole Positions (JSON)", className="form-label"),
                dcc.Textarea(id="gripper-poles", value='[[-0.045,-0.010],[-0.015,-0.010],[0.015,-0.010],[0.045,-0.010],[-0.045,0.010],[-0.015,0.010],[0.015,0.010],[0.045,0.010]]', rows=3, className="form-control form-control-sm font-monospace"),
            ], className="mb-2"),
            html.Div([
                html.Label("Force Curve (JSON: gap_mm -> force_N)", className="form-label"),
                dcc.Textarea(id="gripper-force-curve", value='{"0.0":1070,"0.5":850,"1.0":620,"2.0":340,"3.0":180,"5.0":60}', rows=3, className="form-control form-control-sm font-monospace"),
            ]),
        ], title="Gripper", item_id="gripper-panel")],
        always_open=True, active_item=["gripper-panel"],
    )


def make_billet_section():
    return dbc.Accordion(
        [dbc.AccordionItem([
            dbc.Row([
                dbc.Col([html.Label("Diameter (mm)"), dcc.Input(id="billet-diameter", type="number", value=65, min=1, className="form-control form-control-sm")], md=6),
                dbc.Col([html.Label("Length (mm)"), dcc.Input(id="billet-length", type="number", value=175, min=1, className="form-control form-control-sm")], md=6),
            ], className="mb-2"),
            dbc.Row([
                dbc.Col([html.Label("Weight (kg)"), dcc.Input(id="billet-weight", type="number", value=3.5, min=0.1, step=0.1, className="form-control form-control-sm")], md=6),
                dbc.Col([html.Label("Air Gap (mm)"), dcc.Input(id="billet-air-gap", type="number", value=0, min=0, className="form-control form-control-sm")], md=6),
            ], className="mb-2"),
            dbc.Row([
                dbc.Col([html.Label("Material"), dcc.Dropdown(id="billet-material", options=[
                    {"label": "Forged Steel", "value": "forged_steel"},
                    {"label": "Cast Iron", "value": "cast_iron"},
                    {"label": "Stainless Steel", "value": "stainless"},
                    {"label": "Aluminium", "value": "aluminium"},
                    {"label": "Titanium", "value": "titanium"},
                ], value="forged_steel", className="form-control-sm")], md=6),
                dbc.Col([html.Label("Surface"), dcc.Dropdown(id="billet-surface", options=[
                    {"label": "Clean", "value": "clean"},
                    {"label": "Oily", "value": "oily"},
                    {"label": "Painted", "value": "painted"},
                    {"label": "Rusty", "value": "rusty"},
                    {"label": "Scale", "value": "scale"},
                ], value="clean", className="form-control-sm")], md=6),
            ]),
        ], title="Billet", item_id="billet-panel")],
        always_open=True, active_item=["billet-panel"],
    )


def make_scene_section():
    return dbc.Accordion(
        [dbc.AccordionItem([
            html.Div([
                dcc.Upload(id="scene-ply-upload", children=dbc.Button("Upload .ply point cloud", color="secondary", size="sm", className="w-100"), accept=".ply,.pcd"),
                html.Div(id="scene-file-name", className="text-muted small mt-1"),
            ], className="mb-2"),
            dbc.Checklist(id="scene-use-example", options=[{"label": "Use example scene (datasets/scene.ply)", "value": "example"}], value=["example"], className="mb-2"),
            html.Div([
                html.Label("Point Cloud Resolution", className="form-label"),
                dcc.Slider(id="resolution-slider", min=0, max=3, step=1, value=1,
                    marks={0: {"label": "10K", "style": {"fontSize": "10px"}}, 1: {"label": "50K", "style": {"fontSize": "10px"}}, 2: {"label": "100K", "style": {"fontSize": "10px"}}, 3: {"label": "Full", "style": {"fontSize": "10px"}}},
                    className="mb-2"),
            ]),
        ], title="Scene", item_id="scene-panel")],
        always_open=True, active_item=["scene-panel"],
    )


def make_poses_section():
    return dbc.Accordion(
        [dbc.AccordionItem([
            dbc.Checklist(id="poses-use-example", options=[{"label": "Use example poses (datasets/poses.json)", "value": "example"}], value=[], className="mb-2"),
            html.Div([
                dcc.Upload(id="poses-json-upload", children=dbc.Button("Upload .json poses", color="secondary", size="sm", className="w-100"), accept=".json"),
                html.Div(id="poses-file-name", className="text-muted small mt-1"),
            ], className="mb-2"),
            html.Div([
                html.Label("Or enter poses manually (JSON)", className="form-label"),
                dcc.Textarea(id="poses-manual", rows=6, className="form-control form-control-sm font-monospace",
                    placeholder='{"grasps": [{"position":[x,y,z], "orientation":[x,y,z,w]}, ...]}'),
            ]),
        ], title="Poses", item_id="poses-panel")],
        always_open=True, active_item=["poses-panel"],
    )


def make_bin_section():
    return dbc.Accordion(
        [dbc.AccordionItem([
            dbc.Row([
                dbc.Col([html.Label("Width (mm)"), dcc.Input(id="bin-width", type="number", placeholder="--", className="form-control form-control-sm")], md=4),
                dbc.Col([html.Label("Height (mm)"), dcc.Input(id="bin-height", type="number", placeholder="--", className="form-control form-control-sm")], md=4),
                dbc.Col([html.Label("Depth (mm)"), dcc.Input(id="bin-depth", type="number", placeholder="--", className="form-control form-control-sm")], md=4),
            ]),
        ], title="Bin / Pallet (Optional)", item_id="bin-panel")],
        always_open=False,
    )


def make_input_panel():
    return html.Div([
        html.H5("Input Parameters", className="text-center mb-3"),
        make_gripper_section(),
        make_billet_section(),
        make_scene_section(),
        make_poses_section(),
        make_bin_section(),
    ], style={"overflowY": "auto", "maxHeight": "calc(100vh - 80px)", "padding": "10px"})
