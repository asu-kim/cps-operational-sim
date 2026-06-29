# CPS Operational Simulation

## Overview

This repository contains a working implementation of a cyber-physical systems operational-data pipeline using Lingua Franca (LF), embedded robot logs, drone sensor logs, and MuJoCo replay.

The repository has three main workflows:

1. **Pololu 3Pi+ 2040 robot programs** for track following, bump-maze exploration, encoder/IMU logging, and reusable LF sensor/field reactors: [pololu-3pi](pololu-3pi/README.md)
2. **MuJoCo replay programs** for visualizing recorded Pololu robot traces against measured ground-truth maze and racetrack models: [mujoco-pololu-replay](mujoco-pololu-replay/README.md)
3. **Drone ToF simulation and plotting tools** for replaying five-sensor time-of-flight data, generating RC outputs, and comparing original and modified data: [drone](drone/README.md)

The repository is organized into:

- `pololu-3pi/` for LF programs targeting the Pololu 3Pi+ 2040 robot
- `mujoco-pololu-replay/` for native LF + MuJoCo trace replay
- `drone/` for drone hardware and CSV-based simulation workflows
- `tools/` for Python plotting and analysis utilities
- `data/` for raw logs, processed logs, and generated maps
- `results/` for generated PDFs and experiment outputs
- `mujoco-c/` and `pololu-3pi/pico-sdk/` as submodules

## Prerequisites

You need to download this repository and have the following installed:

- Python 3
- pip
- Lingua Franca compiler, `lfc`
- Lingo, for the MuJoCo LF replay project
- MuJoCo and GLFW, for native graphical replay
- Pico SDK submodule, for RP2040/Pololu embedded builds

After cloning with Git, initialize submodules:

```bash
git submodule update --init --recursive
```

If you are using the zip archive, the submodule directories may be empty. In that case, clone the repository with Git or manually populate:

```bash
git submodule update --init --recursive pololu-3pi/pico-sdk mujoco-c
```

## Library Dependencies

The Python tools use common scientific Python packages:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pandas matplotlib
```

For drone hardware access, install the ToF sensor dependency as needed:

```bash
pip install vl53l1x
```

## Main Workflows

### Pololu 3Pi+ robot programs

The Pololu programs are in `pololu-3pi/src/`.

Important entry points include:

- `BumpMaze_ENCODER_buttons.lf`: bump-maze navigation with encoder, IMU, and CSV field logging
- `TrackFollowSolution_encoder_imu.lf`: track following with encoder odometry, IMU support and CSV logging

Example build commands:

```bash
cd pololu-3pi
lfc src/BumpMaze_ENCODER_buttons.lf
lfc src/TrackFollowSolution_encoder_imu.lf
```

### MuJoCo Pololu replay

The MuJoCo replay programs are in `mujoco-pololu-replay/src/`.

Important entry points include:

- `MazeTraceReplay.lf`: replay bump-maze logs against the maze ground truth
- `RacetrackTraceReplay.lf`: replay oval/racetrack logs
- `RacetrackTraceReplay_square.lf`: replay square-track logs

Example build commands:

```bash
cd mujoco-pololu-replay
rm -rf build
lingo update
lingo build
export LD_LIBRARY_PATH=/opt/mujoco/lib:$LD_LIBRARY_PATH
build/bin/MazeTraceReplay
```

### Drone simulation and plotting

The drone workflow can replay five ToF sensor CSV files and compare generated trajectories.

Example comparison command:

```bash
python3 tools/plot_drone_path_compare.py \
  data/drone/processed/rc-out.csv data/drone/raw-logs/Data1 \
  data/drone/processed/rc-out-2.csv data/drone/raw-logs/Data2 \
  results/drone/simulation/original_vs_modified_overlay.pdf
```

If the ToF data is still zipped, unzip it first:

```bash
cd data/drone/raw-logs
unzip Data1.zip
unzip Data2.zip
```

## Data and Results

- Raw Pololu logs are stored in `data/pololu-3pi/raw-logs/`
- Generated Pololu maps are stored in `data/pololu-3pi/maps/`
- Drone ToF zip files are stored in `data/drone/raw-logs/`
- Drone RC outputs are stored in `data/drone/processed/`
- Final PDFs are stored in `results/`

## Additional Instructions

- Some LF files contain absolute paths from the development machine. If your repository is in a different location, update those paths before building or running.
- The MuJoCo replay programs expect CSV logs with specific names such as `maze_log.csv`, `racetrack_log.csv`, and `square_log.csv` in `mujoco-pololu-replay/data/` unless the LF source is edited.
- Generated build directories such as `bin/`, `build/`, `src-gen/`, and `target/` are intentionally ignored.
- Keep raw logs and generated results separate so experiments can be reproduced.
