#!/usr/bin/env bash
set -euo pipefail
. "$(dirname -- "${BASH_SOURCE[0]}")/scripts/project-env.sh"
exec "$PYTHON" "$PROJECT_ROOT/scripts/verify-r2.py"
