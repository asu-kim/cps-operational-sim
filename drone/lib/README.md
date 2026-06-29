# Drone Shared Library

## Overview

This directory contains the shared LF reactors and helper scripts used by both the drone hardware workflow and the CSV simulation workflow.

## Contents

- `ToFBridgeC.lf`: starts `tof_reader.py` and publishes live ToF distance readings
- `UserLandCmd.lf`: emits a landing command when the user presses `l`
- `avoid_planner_modal.lf`: modal takeoff, cruise, obstacle avoidance, and landing controller
- `msp_sender.lf`: sends MSP RC commands to a serial device or logs commands to CSV
- `tof_reader.py`: Python interface for one ToF sensor stream
- `tof_logger.py`: helper script for recording ToF streams into CSV files

## Prerequisites

See the drone [README.md](../README.md) and repository root [README.md](../../README.md) for the full setup.

Install the Python packages needed by the workflows you are using:

```bash
pip install numpy pandas matplotlib vl53l1x
```

## Usage

These files are imported by:

- `drone/src/test.lf` for hardware execution
- `drone/simulation/src/test.lf` for CSV simulation

## Additional Instructions

- Use relative imports from the LF entry point so the project can run on another machine.
- `msp_sender.lf` can operate in two modes:
  - serial mode when `port` is set to a device such as `/dev/ttyACM0`
  - log-only mode when `port=""`
- `tof_logger.py` can be used to create new `front.csv`, `left.csv`, `right.csv`, `top.csv`, and `bottom.csv` files for later replay.
- Keep the sample period consistent between ToF logging, LF simulation, and RC-output plotting.
