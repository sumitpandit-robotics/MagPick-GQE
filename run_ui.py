#!/usr/bin/env python3
"""
run_ui.py

Launch the MagPick-GQE Dashboard.
Usage: python run_ui.py [--port PORT] [--host HOST]
"""

import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Launch MagPick-GQE Dashboard")
    parser.add_argument("--port", type=int, default=8050, help="Server port (default: 8050)")
    parser.add_argument("--host", default="0.0.0.0", help="Server host (default: 0.0.0.0)")
    args = parser.parse_args()

    from magpick.ui.app import create_app

    app = create_app()
    print(f"\nMagPick-GQE Dashboard v1.1.0")
    print(f"Server: http://{args.host}:{args.port}")
    print(f"Debug mode: ON\n")

    app.run(debug=True, host=args.host, port=args.port)

if __name__ == "__main__":
    main()
