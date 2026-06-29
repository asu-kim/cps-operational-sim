# MuJoCo Ground-Truth and Error-Analysis Tools

## Overview

This directory contains Python tools for generating measured MuJoCo XML ground-truth models and analyzing wall-detection error for the maze case study.

## Contents

- `generate_maze_ground_truth_mjcf.py`: generates the measured maze wall model.
- `generate_fixed_oval_racetrack_mjcf.py`: generates the measured stadium-track ground truth.
- `generate_fixed_square_racetrack_mjcf.py`: generates the measured square-track ground truth.
- `analyze_maze_detected_wall_error.py`: compares detected wall points to the MuJoCo maze ground-truth XML.
- `analyze_maze_wall_detection_error.py`: additional maze wall-detection analysis utility.

## Generate Maze XML

```bash
cd mujoco-pololu-replay
./tools/generate_maze_ground_truth_mjcf.py --out models/pololu_maze_ground_truth.xml --wall-thickness 0.020 --wall-height 0.100
```

## Generate Stadium-Track XML

```bash
./tools/generate_fixed_oval_racetrack_mjcf.py --out models/pololu_fixed_oval_ground_truth.xml
```

## Generate Square-Track XML

```bash
./tools/generate_fixed_square_racetrack_mjcf.py --out models/pololu_fixed_square_ground_truth.xml
```

## Analyze Maze Detected-Wall Error

```bash
./tools/analyze_maze_detected_wall_error.py --detected data/maze_detected_walls.csv --xml models/pololu_maze_ground_truth.xml --wall-thickness 0.020
```

The script reports mean, RMS, and maximum point-to-wall error.

## Notes

- Commit both a generator and the generated XML when physical measurements change.
- Keep generated XML files under `mujoco-pololu-replay/models/`.
- Regenerate replay results after changing ground-truth geometry.
