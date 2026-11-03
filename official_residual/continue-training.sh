#!/usr/bin/env bash
set -euo pipefail
source /mnt/llm/quadruped_dev/activate.sh
cd /mnt/llm/quadruped_dev/official_residual
export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
checkpoint="${1:-runs/official_live_v1/final.zip}"
run_dir="${2:-runs/official_live_$(date +%Y%m%d_%H%M%S)}"
exec python -u train_live.py --steps 8192 --resume "$checkpoint" --run "$run_dir"
