"""
app.py

Dash application creation and server launch.
"""

import dash
import dash_bootstrap_components as dbc

from magpick.ui.layout import make_layout
from magpick.ui.callbacks import register_callbacks


def create_app():
    app = dash.Dash(
        __name__,
        external_stylesheets=[dbc.themes.BOOTSTRAP, dbc.icons.BOOTSTRAP],
        title="MagPick-GQE Dashboard",
        suppress_callback_exceptions=True,
    )
    app.layout = make_layout()
    register_callbacks(app)
    return app


app = create_app()

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=8050)
