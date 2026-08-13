#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MPLBACKEND="QtAgg"
export QT_API="PyQt5"

conda run --no-capture-output -n sota-calibration-sim \
  python "${SCRIPT_DIR}/run.py" \
  --method joint_reference \
  --output "${SCRIPT_DIR}/outputs/joint_noiseless" \
  --show "$@"
