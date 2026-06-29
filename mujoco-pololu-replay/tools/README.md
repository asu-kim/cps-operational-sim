# MuJoCo Ground-Truth Generators

## Overview

This directory contains Python scripts that generate MuJoCo XML ground-truth models for the Pololu replay programs.

## Contents

- `generate_maze_ground_truth_mjcf.py`: generates the measured maze wall model
- `generate_fixed_oval_racetrack_mjcf.py`: generates the measured stadium track ground truth
- `generate_fixed_square_racetrack_mjcf.py`: generates the measured square-track ground truth

## Prerequisites

The scripts use only the Python standard library.

## To Generate the Maze XML

```bash
cd mujoco-pololu-replay
./tools/generate_maze_ground_truth_mjcf.py \
  --out models/pololu_maze_ground_truth.xml \
  --wall-thickness 0.020 \
  --wall-height 0.100
```

## To Generate the Oval/Racetrack XML

```bash
./tools/generate_fixed_oval_racetrack_mjcf.py \
  --out models/pololu_fixed_oval_ground_truth.xml
```

## To Generate the Square-Track XML

```bash
./tools/generate_fixed_square_racetrack_mjcf.py \
  --out models/pololu_fixed_square_ground_truth.xml
```

## Additional Instructions

- Rebuild the LF replay program after changing an XML path or model name in LF source.
- Commit both the generator script and the generated XML when the measured geometry changes.
- Keep generated XML files under `mujoco-pololu-replay/models/`.
