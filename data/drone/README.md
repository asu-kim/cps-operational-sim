# Drone Data

## Overview

This directory stores the data used by the drone obstacle-avoidance workflow.

The raw inputs are five time-of-flight (ToF) sensor streams. The LF drone simulation consumes those ToF streams and writes RC command outputs into `rc-out/`.

## Directory Layout

```text
data/drone/
  raw-logs/
    Data1/
      front.csv
      left.csv
      right.csv
      top.csv
      bottom.csv
    Data2/
      front.csv
      left.csv
      right.csv
      top.csv
      bottom.csv
    Data3/
    Data4/
    Data1.zip
    Data2.zip
  rc-out/
    rc-out.csv
    rc-out-2.csv
    rc-out-3.csv
    rc-out-4.csv
```

## Naming Used in the Experiments

- `Data1` is the original ToF dataset used for the baseline drone obstacle-avoidance run.
- `Data2` is the obstacle-injected dataset used to show how changed sensor data affect the route and RC outputs.
- `rc-out.csv` is the RC output for the first run.
- `rc-out-2.csv` is the RC output for the second run.

Additional datasets, such as `Data3` and `Data4`, are extra replay inputs.

## Expected ToF Files

Every ToF dataset directory must contain:

```text
front.csv
left.csv
right.csv
top.csv
bottom.csv
```

The plotting tools expect those exact filenames.

## Generate a Single-Run Plot

From the repository root:

```bash
python3 tools/plot_drone_path.py data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1
```

## Generate an Original vs Obstacle-Injected Comparison Plot

From the repository root:

```bash
python3 tools/plot_drone_path_compare.py data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1 data/drone/rc-out/rc-out-2.csv data/drone/raw-logs/Data2 results/drone/simulation/original_vs_obstacle_injected_overlay.pdf
```

The comparison plot reconstructs approximate drone trajectories from RC commands and overlays obstacle detections inferred from the ToF logs.

## Notes

- The reconstructed trajectory is approximate because it is integrated from RC commands rather than measured by motion capture.
- Use the plots for relative comparison between original and obstacle-injected data.
- Keep generated RC command logs under `data/drone/rc-out/`.
- Keep generated PDFs under `results/drone/simulation/`.
