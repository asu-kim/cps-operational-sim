# Drone CSV Simulation

## Overview

This directory contains the CSV-based simulation workflow for the Lingua Franca drone demo. It replays recorded ToF sensor data through the avoidance controller and writes RC commands to a CSV log instead of sending them to a flight controller.

This workflow is useful for testing controller changes without flying the drone.

## Directory Structure

- `src/test.lf`: main simulation entry point
- `src/ToFBridgeCSV.lf`: CSV-backed ToF reader used in place of live hardware
- `results/`: simulation-specific notes for generated RC logs and plots

The repository-level data folders are:

- `../../data/drone/raw-logs/`: zipped ToF sensor datasets such as `Data1.zip` and `Data2.zip`
- `../../data/drone/processed/`: RC output logs such as `rc-out.csv` and `rc-out-2.csv`
- `../../results/drone/simulation/`: generated comparison plots

## Prerequisites

See the repository root [README.md](../../README.md) for Python and LF setup.

## To Run the Code

Before building, update the imports in `src/test.lf` if they still point to an old absolute path. The local import block should look like this:

```lf
import PyToF from "ToFBridgeCSV.lf"
import AvoidPlanner from "../../lib/avoid_planner_modal.lf"
import MSPSender from "../../lib/msp_sender.lf"
import UserLandCmd from "../../lib/UserLandCmd.lf"
```

Build and run from the `drone/` directory:

```bash
cd drone
lfc simulation/src/test.lf
./simulation/bin/test
```

If `src/test.lf` is still configured to read `simulation/data/Data4`, create that directory or edit the paths before running. For example, from the repository root:

```bash
mkdir -p drone/simulation/data
unzip data/drone/raw-logs/Data1.zip -d drone/simulation/data
```

Then set the five `path=` values in `drone/simulation/src/test.lf` to `simulation/data/Data1/bottom.csv`, `front.csv`, `right.csv`, `left.csv`, and `top.csv`, or change them to another dataset.

## Additional Instructions

- `ToFBridgeCSV.lf` reads CSV files configured in `simulation/src/test.lf`.
- The five expected sensor files are `bottom.csv`, `front.csv`, `right.csv`, `left.csv`, and `top.csv`.
- `MSPSender(port="")` disables serial transmission and writes RC commands to a CSV file.
- Change the `log_path` value in `src/test.lf` when you want to preserve multiple simulation runs.
- Launch from a directory that matches the relative CSV paths configured in `src/test.lf`.

## Verification Instructions

From the repository root, generate a path plot from the RC outputs and matching ToF data:

```bash
python3 tools/plot_drone_path.py \
  data/drone/processed/rc-out.csv \
  data/drone/raw-logs/Data1
```

To compare two runs from the repository root:

```bash
python3 tools/plot_drone_path_compare.py \
  data/drone/processed/rc-out.csv data/drone/raw-logs/Data1 \
  data/drone/processed/rc-out-2.csv data/drone/raw-logs/Data2 \
  results/drone/simulation/original_vs_modified_overlay.pdf
```
