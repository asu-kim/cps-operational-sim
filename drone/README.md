# Lingua Franca Drone Obstacle-Avoidance Workflow

## Overview

This directory contains the Lingua Franca (LF) drone workflow used for the drone obstacle-avoidance case study.

The workflow has two modes:

1. **Hardware-oriented mode** using live ToF readings and MSP RC output.
2. **CSV simulation mode** using recorded ToF readings and log-only RC output.

The CSV simulation mode is the one used to compare original ToF data with obstacle-injected data.

## Directory Structure

```text
drone/
  src/
    test.lf                     # hardware-oriented entry point
    DroneBridgeC.lf
    avoid_planner_modal.lf
  lib/
    ToFBridgeC.lf               # live ToF bridge
    avoid_planner_modal.lf      # obstacle-avoidance planner
    msp_sender.lf               # MSP sender or CSV logger
    UserLandCmd.lf
    tof_reader.py
    tof_logger.py
  simulation/
    src/
      drone.lf                  # CSV simulation entry point in this repo
      ToFBridgeCSV.lf
```

## Dependencies

For plotting and CSV analysis:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pandas matplotlib
```

For live ToF hardware, install the sensor package required by your hardware setup.

## CSV Simulation Workflow

The simulation replays five ToF CSV streams through the avoid planner and writes RC commands to a CSV file.

The current simulation entry point is:

```text
drone/simulation/src/drone.lf
```

Build from the `drone/` directory:

```bash
cd drone
lfc simulation/src/drone.lf
```

Then run the generated executable from the directory expected by the LF-generated output.

The simulation source currently chooses the ToF input files and RC output path inside the LF parameters. Update these paths when switching between `Data1`, `Data2`, `Data3`, or `Data4`.

## Hardware-Oriented Workflow

The hardware-oriented source is:

```text
drone/src/test.lf
```

Before using it with a physical drone, check:

- ToF sensor wiring and addresses
- serial device used by `MSPSender`
- safety and landing behavior
- whether LF imports use repository-local paths

## Plotting Outputs

Use the repository-level plotting scripts after generating RC output logs:

```bash
python3 tools/plot_drone_path_compare.py data/drone/rc-out/rc-out.csv data/drone/raw-logs/Data1 data/drone/rc-out/rc-out-2.csv data/drone/raw-logs/Data2 results/drone/simulation/original_vs_obstacle_injected_overlay.pdf
```

## Notes

- The LF simulation should be treated as a replay of recorded operational data.
- The output RC logs are stored under `data/drone/rc-out/` in the repository-level data layout.
- Use the terms **original data** and **obstacle-injected data** for the two-run comparison.
