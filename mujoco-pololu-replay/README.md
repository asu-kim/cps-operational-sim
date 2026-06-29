# MuJoCo Pololu Replay

## Overview

This directory contains native LF + MuJoCo replay programs for Pololu operational data.

The replay programs load CSV logs from physical robot executions, move a simple MuJoCo robot body along the recorded pose trace, and draw measured ground-truth geometry for comparison.

The supported replay cases are:

- square-track following
- stadium-track following
- bump-based maze exploration

## Main Files

```text
mujoco-pololu-replay/
  Lingo.toml
  src/
    MazeTraceReplay.lf
    RacetrackTraceReplay_oval.lf
    RacetrackTraceReplay_square.lf
  data/
    maze_log.csv
    racetrack_log.csv
    square_log.csv
    maze_detected_walls.csv
    maze_detected_walls_error.csv
  models/
    pololu_maze_ground_truth.xml
    pololu_fixed_oval_ground_truth.xml
    pololu_fixed_square_ground_truth.xml
  tools/
    generate_maze_ground_truth_mjcf.py
    generate_fixed_oval_racetrack_mjcf.py
    generate_fixed_square_racetrack_mjcf.py
    analyze_maze_detected_wall_error.py
    analyze_maze_wall_detection_error.py
```

## Build

From this directory:

```bash
cd mujoco-pololu-replay
rm -rf build
lingo update
lingo build
```

If MuJoCo is installed under `/opt/mujoco`, set:

```bash
export LD_LIBRARY_PATH=/opt/mujoco/lib:$LD_LIBRARY_PATH
```

## Run

```bash
build/bin/MazeTraceReplay
build/bin/RacetrackTraceReplayOval
build/bin/RacetrackTraceReplaySquare
```

## Expected CSV Inputs

The replay programs use fixed filenames under `mujoco-pololu-replay/data/`:

```text
maze_log.csv       # bump-based maze exploration
racetrack_log.csv  # stadium-track following
square_log.csv     # square-track following
```

Copy or rename the desired raw logs before replaying.

## Ground-Truth XML Generation

Refresh the ground-truth XML files with:

```bash
./tools/generate_maze_ground_truth_mjcf.py --out models/pololu_maze_ground_truth.xml

./tools/generate_fixed_oval_racetrack_mjcf.py --out models/pololu_fixed_oval_ground_truth.xml

./tools/generate_fixed_square_racetrack_mjcf.py --out models/pololu_fixed_square_ground_truth.xml
```

## Maze Wall-Detection Error

The maze replay can visualize detected wall locations generated from bump events. The analysis tools compare detected wall points against the nearest ground-truth wall segment:

```bash
./tools/analyze_maze_detected_wall_error.py --detected data/maze_detected_walls.csv --xml models/pololu_maze_ground_truth.xml --wall-thickness 0.020
```

## Notes

- The maze model is shifted so the robot start is `(0, 0)`.
- Track-following replay uses measured square and stadium ground-truth geometry.
- Replay accuracy depends on the logged pose fields: `x_m`, `y_m`, and `theta_deg`.
- If the repository path changes, check the absolute CSV or XML paths inside the LF replay files.
