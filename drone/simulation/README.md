# Overview

This directory contains the CSV-based simulation workflow for the Lingua Franca drone demo. It replays recorded ToF measurements through the same avoidance logic used by the hardware workflow and logs the resulting RC commands for offline analysis.

## Directory Structure

- `src/test.lf`: main simulation entry point
- `src/ToFBridgeCSV.lf`: CSV-backed ToF reader used in place of live hardware
- `data/`: sample ToF input CSV files collected from the drone using ./lib
- `results/`: RC logs, plots, and plotting utilities

# Prerequisites

See the repository root [README.md](../README.md) for the shared software setup and Python dependencies.

### To Run the Code

```bash
lfc simulation/src/test.lf
./simulation/bin/test
```

### Additional Instructions

- Before running `lfc`, update the hard-coded absolute import paths in `simulation/src/test.lf` so they point to the local files in this repository's `lib/` directory.
- In this file, `ToFBridgeCSV.lf` is already local to `simulation/src/`, so that import can stay unchanged.

- For example, change:

```lf
import AvoidPlanner from "/mnt/e/PhD/lf-demos/lf-drone/lib/avoid_planner_modal.lf"
```

to:

```lf
import AvoidPlanner from "../../lib/avoid_planner_modal.lf"
```

- The full import block in `simulation/src/test.lf` should look like this:

```lf
import PyToF from "ToFBridgeCSV.lf"
import AvoidPlanner from "../../lib/avoid_planner_modal.lf"
import MSPSender from "../../lib/msp_sender.lf"
import UserLandCmd from "../../lib/UserLandCmd.lf"
```

- The current `src/test.lf` file reads sensor data from `simulation/data/`, `bottom.csv`, `front.csv`, `right.csv`, `left.csv`, and `top.csv`.
- The simulation sets `port=""` in `MSPSender`, so it runs in log-only mode and does not require a real flight controller.
- The current RC output log path is `simulation/results/rc-out.csv`. You can change that value in `src/test.lf` if you want to create a new log file.
- Launch the final executable from the repository root so the configured CSV and log paths resolve correctly.

### Verification Instructions

After the simulation finishes, you can generate a path plot from the RC outputs and the ToF CSV files:

```bash
python3 simulation/results/plot_drone_path.py simulation/results/rc-out-file simulation/data
```

The output PDF is written into `simulation/results/`.