"""
layout.py

Full UI layout assembly for the MagPick-GQE Dashboard.
"""

import dash_bootstrap_components as dbc
from dash import dcc, html

from magpick.ui.inputs import make_input_panel
from magpick.ui.viewport_3d import make_3d_viewport
from magpick.ui.results import make_summary_cards, make_comparison_chart, make_candidate_list, make_candidate_detail, make_download_bar


CUSTOM_CSS = {
    "body": {
        "fontFamily": "'Segoe UI', -apple-system, BlinkMacSystemFont, sans-serif",
        "backgroundColor": "#f0f2f5",
    },
    "sidebar": {
        "backgroundColor": "#ffffff",
        "borderRight": "1px solid #dee2e6",
        "padding": "0",
        "overflowY": "auto",
        "maxHeight": "calc(100vh - 65px)",
    },
    "main-area": {
        "padding": "15px",
        "overflowY": "auto",
        "maxHeight": "calc(100vh - 65px)",
    },
}


def make_layout():
    return dbc.Container([
        # Store for evaluation results
        dcc.Store(id="eval-store", storage_type="memory"),

        # Navbar
        dbc.Navbar(
            dbc.Container([
                dbc.Row([
                    dbc.Col(html.Span("MagPick-GQE", className="navbar-brand fw-bold fs-5"), width="auto"),
                    dbc.Col(html.Span("Industrial Grasp Quality Dashboard", className="text-muted small"), width="auto"),
                ], align="center", className="g-0"),
                dbc.Col([
                    dbc.Button([
                        html.I(className="bi bi-play-fill me-1"), "Run Evaluation"
                    ], id="run-btn", color="primary", size="lg", className="px-4"),
                ], width="auto"),
            ], fluid=True),
            color="white",
            className="mb-0 shadow-sm",
            style={"borderBottom": "2px solid #3498db"},
        ),

        # Main content
        dbc.Row([
            # Left sidebar - Input Panel
            dbc.Col(make_input_panel(), xs=12, md=3, lg=3, style=CUSTOM_CSS["sidebar"]),

            # Right main area
            dbc.Col([
                # 3D Viewport
                make_3d_viewport(),
                html.Hr(),

                # Summary Dashboard
                make_summary_cards(),

                # Download bar
                make_download_bar(),

                # Comparison Chart + Candidate List
                dbc.Row([
                    dbc.Col(make_comparison_chart(), md=7),
                    dbc.Col(make_candidate_list(), md=5),
                ], className="mb-3"),

                html.Hr(),

                # Candidate Detail
                make_candidate_detail(),

                # Footer
                html.Hr(),
                html.Div([
                    html.Small("MagPick-GQE v1.1.0 | Grasp Quality Evaluation Framework", className="text-muted"),
                ], className="text-center py-2"),
            ], xs=12, md=9, lg=9, style=CUSTOM_CSS["main-area"]),
        ], className="g-0", style={"minHeight": "calc(100vh - 65px)"}),

    ], fluid=True, style={"maxWidth": "100%", "padding": "0"})
