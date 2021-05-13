#!/usr/bin/env bash
# scripts/run_tests.sh: Headless verification runner for Beacon

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== Verifying No Em Dash Policy ==="
python3 "${ROOT_DIR}/scripts/check_no_em_dash.py"

echo ""
echo "=== Locating Godot Executable ==="
GODOT_BIN=""
if command -v godot &> /dev/null; then
    GODOT_BIN="godot"
elif [ -f "/Applications/Godot.app/Contents/MacOS/Godot" ]; then
    GODOT_BIN="/Applications/Godot.app/Contents/MacOS/Godot"
elif command -v godot4 &> /dev/null; then
    GODOT_BIN="godot4"
else
    echo "Error: Godot engine executable not found in PATH or /Applications."
    exit 1
fi

echo "Using Godot binary: ${GODOT_BIN}"
"${GODOT_BIN}" --version

echo ""
echo "=== Launching Headless Test Suite ==="
cd "${ROOT_DIR}"
"${GODOT_BIN}" --headless --no-header -s tests/test_runner.gd
