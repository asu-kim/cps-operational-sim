# Analysis and Plotting Tools

## Overview

This directory contains Python plotting tools for the drone obstacle-avoidance workflow.

## Contents

- `plot_drone_path.py`: reconstructs an approximate drone path from one RC log and one ToF sensor directory.
- `plot_drone_path_compare.py`: overlays two drone runs on the same axes and labels them as original and obstacle-injected data when configured in the script.

## Dependencies

From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pandas matplotlib
```

## Single-Run Plot

```bash
python3 tools/plot_drone_path.py data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1
```

## Two-Run Comparison Plot

```bash
python3 tools/plot_drone_path_compare.py data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1 data/drone/rc-out/rc-out-2.csv data/drone/raw-logs/Data2 results/drone/simulation/original_vs_obstacle_injected_overlay.pdf
```

## Notes

- Each ToF directory must contain `front.csv`, `left.csv`, `right.csv`, `top.csv`, and `bottom.csv`.
- The reconstructed drone path is approximate because it is integrated from RC outputs.
- Use these plots for relative comparison between operational-data runs.
