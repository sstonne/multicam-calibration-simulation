#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MPLBACKEND="QtAgg"
export QT_API="PyQt5"

conda run --no-capture-output -n sota-calibration-sim \
  python "${SCRIPT_DIR}/tsai_combined_demo.py" \
  --output "${SCRIPT_DIR}/outputs/tsai_combined_noiseless" \
  --show "$@"
