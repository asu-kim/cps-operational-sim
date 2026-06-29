# Analysis and Plotting Tools

## Overview

This directory contains Python scripts for plotting and comparing operational data from the drone and Pololu robot workflows.

## Contents

- `plot_drone_path.py`: reconstructs an approximate drone path from one RC log and one ToF sensor directory
- `plot_drone_path_compare.py`: overlays two drone runs on the same axes

## Prerequisites

Install the plotting dependencies from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pandas matplotlib
```

## Drone Single-Run Plot

```bash
python3 tools/plot_drone_path.py \
  data/drone/rc-out/rc-out.csv \
  data/drone/raw-logs/Data1
```

## Drone Two-Run Comparison Plot

```bash
python3 tools/plot_drone_path_compare.py \
  data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1 \
  data/drone/rc-out/rc-out-2.csv data/drone/raw-logs/Data2 \
  results/drone/simulation/original_vs_modified_overlay.pdf
```

## Additional Instructions

- The drone plotting scripts expect ToF directories with `front.csv`, `left.csv`, `right.csv`, `top.csv`, and `bottom.csv`.
