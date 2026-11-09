#!/bin/sh
set -eu
cd "$(dirname "$0")"
python3 -m venv .build-venv
.build-venv/bin/python -m pip install -r requirements-build.txt
exec .build-venv/bin/python build_app.py --target macos "$@"