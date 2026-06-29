# Results

## Overview

This directory stores generated plots, reports, and other experiment outputs. Files here should be reproducible from the raw logs in `data/` and the scripts in `tools/` or `mujoco-pololu-replay/tools/`.

## Directory Structure

- `drone/simulation/`: drone path plots and original-vs-modified comparison PDFs

## Regenerating Drone Results

From the repository root:

```bash
python3 tools/plot_drone_path_compare.py \
  data/drone/processed/rc-out.csv data/drone/raw-logs/Data1 \
  data/drone/processed/rc-out-2.csv data/drone/raw-logs/Data2 \
  results/drone/simulation/original_vs_modified_overlay.pdf
```

## Additional Instructions

- Keep generated figures in `results/`.
- Keep cleaned CSVs and map-specific intermediate files in `data/` when they are inputs to additional analysis.
- Include enough information in filenames to identify the run or dataset being compared.
