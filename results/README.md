# Results

## Overview

This directory stores generated figures and reports. Files here should be reproducible from logs in `data/` and scripts in `tools/` or `mujoco-pololu-replay/tools/`.

## Directory Structure

```text
results/
  drone/
    simulation/
      *.pdf
```

## Drone Obstacle-Avoidance Plots

Generate a comparison between original and obstacle-injected data:

```bash
python3 tools/plot_drone_path_compare.py data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1 data/drone/rc-out/rc-out-2.csv data/drone/raw-logs/Data2 results/drone/simulation/original_vs_obstacle_injected_overlay.pdf
```

## Notes

- Keep generated plots in `results/`.
- Keep raw logs in `data/`.
- Use descriptive filenames that identify the dataset or experiment being compared.
- Regenerate figures after changing plotting scripts, data, or ground-truth geometry.
