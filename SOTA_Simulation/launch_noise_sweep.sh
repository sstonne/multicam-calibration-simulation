#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export MPLBACKEND="QtAgg"
export QT_API="PyQt5"

conda run --no-capture-output -n sota-calibration-sim \
  python "${SCRIPT_DIR}/tsai_noise_sweep.py" \
  --output "${SCRIPT_DIR}/outputs/opencv_noise_sweep" \
  --noise-mm 0 1 3 5 \
  --methods all \
  --trials 30 \
  --show "$@"
