# Lingua Franca Drone Workflow

## Overview

This directory contains the Lingua Franca drone workflow for running the avoidance controller with either live time-of-flight sensors or recorded CSV data.

The workflow has two execution modes:

1. **Hardware mode** using live ToF sensors and MSP RC output: `src/test.lf`
2. **Simulation mode** using recorded ToF CSV files and log-only RC output: [simulation](simulation/README.md)

The shared LF reactors and Python helper scripts are stored in [lib](lib/README.md).

## Directory Structure

- `src/test.lf`: main hardware demo using live ToF sensors
- `src/DroneBridgeC.lf`: standalone serial RC bridge experiment
- `src/avoid_planner_modal.lf`: avoidance planner reactor
- `lib/`: shared LF reactors and Python ToF/MSP helper scripts
- `simulation/`: CSV replay workflow for testing the same avoidance logic offline

## Prerequisites

See the repository root [README.md](../README.md) for Python, LF, and package setup.

For hardware mode, the system also expects:

- A configured drone platform or compatible test rig
- Five ToF sensor streams: `front`, `left`, `right`, `top`, and `bottom`
- Access to the serial device used for RC/MSP output, for example `/dev/ttyACM0`

## Library Dependencies

For live ToF access:

```bash
pip install vl53l1x
```

For plotting and simulation:

```bash
pip install numpy pandas matplotlib
```

## To Run the Hardware Code

Before building, update the imports in `src/test.lf` if they still point to an old absolute path. The local import block should use the repository-local `lib/` directory:

```lf
import PyToF from "../lib/ToFBridgeC.lf"
import AvoidPlanner from "../lib/avoid_planner_modal.lf"
import MSPSender from "../lib/msp_sender.lf"
import UserLandCmd from "../lib/UserLandCmd.lf"
```

Build and run from the repository root:

```bash
lfc drone/src/test.lf
./drone/bin/test
```

## Additional Instructions

- Check the ToF bus numbers and I2C addresses in `src/test.lf` before flying.
- Check `MSPSender(port="/dev/ttyACM0")` and change the serial port if your flight controller uses a different device.
- Run the executable from the repository root because `ToFBridgeC.lf` launches `python3 ./lib/tof_reader.py` relative to the working directory.
- Press `l` while the program is running to request landing through `UserLandCmd`.
- Use the CSV simulation workflow first when testing controller changes.
