#!/usr/bin/env bash
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
bash "$project/scripts/prepare.sh"
bash "$project/buildkernel.sh"
bash "$project/buildmodules.sh"
bash "$project/verify.sh"
