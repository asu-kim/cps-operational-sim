# Overview

This directory contains the shared Lingua Franca reactors and helper scripts used by both the hardware and simulation workflows.

## Contents

- `ToFBridgeC.lf`: spawns `lib/tof_reader.py` and publishes live ToF distance readings
- `UserLandCmd.lf`: emits a landing command when the user presses `l`
- `avoid_planner_modal.lf`: modal takeoff, cruise, avoidance, and landing controller
- `msp_sender.lf`: sends MSP RC commands or logs them to CSV
- `tof_reader.py`: Python interface to the VL53L1X sensor
- `tof_logger.py`: helper script to log one or more ToF streams into CSV files

# Prerequisites

See the repository root [README.md](../README.md) for the required toolchain and Python libraries.

### To Run the Code

These LF files are imported by the programs in:

- `drone/src/test.lf`
- `simulation/src/test.lf`

### Additional Instructions

- Before running `lfc`, update the old absolute imports in `drone/src/test.lf` and `simulation/src/test.lf` so they point to this local `lib/` directory.
- `ToFBridgeC.lf` launches `python3 ./lib/tof_reader.py`, so executables that depend on it should be started from the repository root.
- `msp_sender.lf` supports both serial mode and log-only mode. Setting `port=""` disables serial transmission and keeps only the logging behavior.
- `tof_logger.py` can be used to record live ToF sensor data into CSV files for later replay in the simulation workflow.