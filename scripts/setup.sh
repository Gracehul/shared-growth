#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

HARDWARE=false
if [[ "${1:-}" == "--hardware" ]]; then
  HARDWARE=true
elif [[ -n "${1:-}" ]]; then
  echo "Usage: bash scripts/setup.sh [--hardware]" >&2
  exit 2
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
"$PYTHON_BIN" -c "import sys; assert sys.version_info >= (3, 10), 'Python 3.10+ required'; print(sys.version)"

if [[ ! -d .venv ]]; then
  "$PYTHON_BIN" -m venv .venv
fi

VENV_PYTHON="$PROJECT_ROOT/.venv/bin/python"
"$VENV_PYTHON" -m pip install --upgrade pip

if [[ "$HARDWARE" == true ]]; then
  "$VENV_PYTHON" -m pip install -e ".[hardware]"
else
  "$VENV_PYTHON" -m pip install -e .
fi

"$VENV_PYTHON" -m drawing_robot.preflight

echo "Setup complete. Activate with: source .venv/bin/activate"
if [[ "$HARDWARE" == true ]]; then
  echo "Hardware packages are installed. No robot connection was opened."
fi
