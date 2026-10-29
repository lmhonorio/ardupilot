#!/usr/bin/env bash
set -euo pipefail
ROVER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${PYTHON38:-python3.8}" "$ROVER_DIR/python/missions/run.py" "$@"
