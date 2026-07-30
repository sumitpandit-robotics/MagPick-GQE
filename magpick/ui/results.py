"""
results.py

Results dashboard and candidate detail panels.
"""

import dash_bootstrap_components as dbc
from dash import dcc, html
import plotly.graph_objects as go

from magpick.ui.utils import score_to_color
from magpick.utils.visualization import render_force_vector_svg, compute_radar_data, generate_recommendations


def make_summary_cards():
    return dbc.Row([
        dbc.Col(dbc.Card([dbc.CardBody([html.H6("Best Score", className="text-muted mb-1"), html.H3(id="card-best-score", children="--")], className="text-center py-2")], outline=True), md=2),
        dbc.Col(dbc.Card([dbc.CardBody([html.H6("Compatibility", className="text-muted mb-1"), html.H3(id="card-compat", children="--")], className="text-center py-2")], outline=True), md=2),
        dbc.Col(dbc.Card([dbc.CardBody([html.H6("Candidates", className="text-muted mb-1"), html.H3(id="card-total", children="--")], className="text-center py-2")], outline=True), md=2),
        dbc.Col(dbc.Card([dbc.CardBody([html.H6("Accepted", className="text-muted mb-1"), html.H3(id="card-passed", children="--")], className="text-center py-2")], color="success", outline=True), md=2),
        dbc.Col(dbc.Card([dbc.CardBody([html.H6("Rejected", className="text-muted mb-1"), html.H3(id="card-failed", children="--")], className="text-center py-2")], color="danger", outline=True), md=2),
        dbc.Col(dbc.Card([dbc.CardBody([html.H6("Pass Rate", className="text-muted mb-1"), html.H3(id="card-rate", children="--")], className="text-center py-2")], outline=True), md=2),
    ], className="mb-3")


def make_comparison_chart():
    return dcc.Graph(id="comparison-chart", style={"height": "280px"}, config={"displayModeBar": False})


def make_candidate_list():
    return html.Div([html.H5("Candidate Rankings", className="mb-2"), html.Div(id="candidate-list-body")])


def make_candidate_detail():
    return html.Div(id="candidate-detail", children=[
        html.Div([html.H5("Select a candidate", className="text-muted"), html.P("Click on a candidate in the 3D view or list", className="text-muted")], className="text-center py-5"),
    ])


def render_candidate_list(candidates, scores, selected_idx=None):
    header = dbc.Row([
        dbc.Col(html.Strong("#"), width=1), dbc.Col(html.Strong("Score"), width=2),
        dbc.Col(html.Strong("Status"), width=2),
    ], className="py-1 mb-1 border-bottom")

    rows = []
    for i, (cand, score) in enumerate(zip(candidates, scores)):
        color = score_to_color(score)
        status = "PASS" if score > 0 else "FAIL"
        bg = "#e8f4fd" if i == selected_idx else "#f8f9fa"
        rows.append(html.Div(
            dbc.Row([
                dbc.Col(html.Span(f"#{i+1}", className="fw-bold"), width=1),
                dbc.Col(html.Span(f"{score:.3f}", style={"color": color, "fontWeight": "bold"}), width=2),
                dbc.Col(dbc.Badge(status, color="success" if score > 0 else "danger"), width=2),
            ], className="py-1"),
            id={"candidate-row": i}, n_clicks=0,
            style={"border": "1px solid #dee2e6", "borderLeft": f"3px solid {color}", "borderRadius": "4px",
                   "marginBottom": "2px", "cursor": "pointer", "backgroundColor": bg, "padding": "2px 6px"},
        ))
    return html.Div([header] + rows)


def render_candidate_detail(candidate, evaluator_results, score, snapshots=None):
    if candidate is None:
        return html.Div([html.H5("Select a candidate", className="text-center text-muted"), html.P("Click on a candidate", className="text-center text-muted")], className="py-5")

    # Evaluator table
    ev_rows = []
    for ev in evaluator_results:
        cls = "text-success" if ev.passed else "text-danger"
        icon = "\u2714" if ev.passed else "\u2718"
        ev_rows.append(html.Tr([
            html.Td(icon, className=cls, style={"width": "30px"}),
            html.Td(ev.name), html.Td(f"{ev.score:.3f}"),
            html.Td(ev.reason, className="small text-muted"),
        ]))
    ev_table = dbc.Table([html.Thead(html.Tr([html.Th("Evaluator"), html.Th("Score"), html.Th("Reason")])), html.Tbody(ev_rows)], bordered=True, hover=True, size="sm")

    # Radar chart
    radar_div = html.Div("No data")
    if evaluator_results:
        rd = compute_radar_data(evaluator_results)
        radar_fig = go.Figure(go.Scatterpolar(r=rd["scores"] + [rd["scores"][0]], theta=rd["labels"] + [rd["labels"][0]], fill="toself", fillcolor="rgba(52,152,219,0.2)", line=dict(color="#3498db")))
        radar_fig.update_layout(polar=dict(radialaxis=dict(visible=True, range=[0, 1])), showlegend=False, margin=dict(l=40, r=40, t=20, b=20), height=280)
        radar_div = dcc.Graph(figure=radar_fig, config={"displayModeBar": False})

    # Force diagram
    mag_ev = next((ev for ev in evaluator_results if ev.name == "Magnetic"), None)
    force_div = html.Div("No magnetic data")
    if mag_ev:
        d = mag_ev.details
        force_svg = render_force_vector_svg(holding_force=d.get("holding_force", 0), required_force=d.get("required_force", 0), safety_factor=d.get("safety_factor", 0))
        force_div = html.Div([html.H6("Force Analysis"), html.Div(dangerouslySetInnerHTML={"__html": force_svg})])

    # Recommendations
    recs = generate_recommendations(evaluator_results)
    rec_items = []
    for rec in recs:
        risk_cls = {"High": "danger", "Medium": "warning", "Low": "success"}.get(rec["risk"], "secondary")
        rec_items.append(html.Div([html.Span(rec["icon"], className="me-1"), html.Strong(rec["evaluator"]), " \u2014 ", rec["reason"], html.Br(), html.Small(f"Risk: ", html.Span(rec["risk"], className=f"text-{risk_cls} fw-bold"), " | Suggestion: ", rec["suggestion"], className="text-muted")], className="mb-2 pb-2 border-bottom"))

    # Snapshots
    snap_div = html.Div()
    if snapshots:
        snap_items = []
        labels = {"isometric": "Isometric", "top": "Top", "front": "Front", "side": "Side"}
        for vn, b64 in snapshots.items():
            snap_items.append(html.Div([html.Small(labels.get(vn, vn), className="text-muted"), html.Img(src=f"data:image/png;base64,{b64}", style={"width": "100%", "borderRadius": "4px", "border": "1px solid #dee2e6"})], className="text-center"))
        snap_div = html.Div([html.H6("Multi-View Snapshots"), dbc.Row([dbc.Col(s, xs=6, md=3) for s in snap_items], className="mb-3")])

    return html.Div([
        dbc.Row([dbc.Col([html.H5([html.Span(f"Candidate #{getattr(candidate, 'rank', '?')}", className="me-2"), dbc.Badge(f"Score: {score:.3f}", color="success" if score > 0 else "danger", className="me-2"), dbc.Badge("PASS" if score > 0 else "FAIL", color="success" if score > 0 else "danger")])])]),
        html.Hr(),
        snap_div,
        dbc.Row([
            dbc.Col([html.H6("Evaluator Results"), ev_table], md=6),
            dbc.Col([html.H6("Score Radar"), radar_div], md=3),
            dbc.Col([html.H6("Force Analysis"), force_div], md=3),
        ]),
        html.Hr(),
        html.Div([html.H6("Engineering Recommendations")] + rec_items if rec_items else [html.P("No recommendations \u2014 all evaluators passed.", className="text-muted")]),
    ])
