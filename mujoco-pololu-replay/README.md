# MuJoCo Pololu Replay

## Overview

This directory contains native Lingua Franca + MuJoCo replay programs for visualizing recorded Pololu 3Pi+ robot data against measured ground-truth tracks and maze walls.

The replay programs do not control the physical robot. They load CSV logs, move a simple MuJoCo robot body along the recorded trajectory, and draw the ground-truth geometry for comparison.

## Main Files

- `src/MazeTraceReplay.lf`: replays bump-maze data against `models/pololu_maze_ground_truth.xml`
- `src/RacetrackTraceReplay_oval.lf`: replays stadium track data against `models/pololu_fixed_oval_ground_truth.xml`
- `src/RacetrackTraceReplay_square.lf`: replays square-track data against `models/pololu_fixed_square_ground_truth.xml`
- `models/`: generated MuJoCo XML ground-truth models
- `tools/`: Python generators for measured maze, oval, and square ground-truth XML files
- `Lingo.toml`: Lingo package definition and app targets

## Prerequisites

You need:

- Lingo
- Lingua Franca C target support
- MuJoCo
- GLFW
- `mujoco-c` submodule initialized at `../mujoco-c`

Initialize the MuJoCo LF dependency:

```bash
git submodule update --init --recursive mujoco-c
```

## To Build the Replay Programs

From this directory:

```bash
cd ~/cps-operational-sim/pololu-3pi/mujoco-pololu-replay
rm -rf build
lingo update
lingo build
```

If MuJoCo is installed under `/opt/mujoco`, set:

```bash
export LD_LIBRARY_PATH=/opt/mujoco/lib:$LD_LIBRARY_PATH
```

Run one of the replay programs:

```bash
build/bin/MazeTraceReplay
build/bin/RacetrackTraceReplayOval
build/bin/RacetrackTraceReplaySquare
```

## CSV Input Logs

The LF replay sources currently load fixed CSV paths. Before running, copy or rename logs as needed:

```bash
mkdir -p data
cp ../data/pololu-3pi/raw-logs/pololu_bump_encoder_maze_20260617_163722.csv data/maze_log.csv
cp ../data/pololu-3pi/raw-logs/pololu_line_encoder_track_follow_20260610_140926.csv data/racetrack_log.csv
cp ../data/pololu-3pi/raw-logs/pololu_line_encoder_track_follow_20260610_142100.csv data/square_log.csv
```

## Ground-Truth XML Generation

Generate or refresh measured ground-truth XML files with:

```bash
./tools/generate_maze_ground_truth_mjcf.py \
  --out models/pololu_maze_ground_truth.xml

./tools/generate_fixed_oval_racetrack_mjcf.py \
  --out models/pololu_fixed_oval_ground_truth.xml

./tools/generate_fixed_square_racetrack_mjcf.py \
  --out models/pololu_fixed_square_ground_truth.xml
```

## Additional Instructions

- The maze model uses measured wall coordinates shifted so the robot start is `(0, 0)`.
- The racetrack models draw the measured ground-truth path using black MuJoCo geoms.
- Replay accuracy depends on the CSV pose fields, especially `x_m`, `y_m`, and `theta_deg`.
- Use the Python plotting tools in `../tools/` to inspect the CSV logs before replaying them in MuJoCo.
