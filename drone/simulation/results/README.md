# Drone Simulation Results

## Overview

This directory is reserved for RC logs and plots produced by the drone CSV simulation workflow. In the current repository layout, most processed drone logs are stored in `data/drone/processed/` and final PDFs are stored in `results/drone/simulation/`.

## Typical Outputs

- `rc-out.csv`: RC command log from a simulation run
- `rc-out-2.csv`, `rc-out-3.csv`, `rc-out-4.csv`: additional simulation runs
- `*-path-obstacles.pdf`: single-run path and obstacle plot
- `*_overlay.pdf`: two-run trajectory comparison plot

## To Generate a Single-Run Plot

From the repository root:

```bash
python3 tools/plot_drone_path.py \
  data/drone/processed/rc-out.csv \
  data/drone/raw-logs/Data1
```

## To Generate a Two-Run Comparison Plot

```bash
python3 tools/plot_drone_path_compare.py \
  data/drone/processed/rc-out.csv data/drone/raw-logs/Data1 \
  data/drone/processed/rc-out-2.csv data/drone/raw-logs/Data2 \
  results/drone/simulation/original_vs_modified_overlay.pdf
```

## Additional Instructions

- Make sure each RC log is paired with the matching ToF sensor directory.
- The ToF directory must contain `front.csv`, `left.csv`, `right.csv`, `top.csv`, and `bottom.csv`.
- The plotting tools reconstruct an approximate trajectory from RC commands, so the plots are best used for relative comparison rather than absolute localization.
