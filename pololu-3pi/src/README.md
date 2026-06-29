# Pololu LF Source Programs

## Overview

This directory contains LF application programs for the Pololu 3Pi+ 2040 robot.

## Main Programs

- `BumpMaze_ENCODER_buttons.lf`: bump-based maze exploration with button control, bump sensors, encoders, IMU fields, odometry, and CSV logging.
- `TrackFollowSolution_encoder_imu.lf`: track-following experiment with line-sensor readings, encoder odometry, IMU heading, controller state, motor commands, and CSV logging.
- `lib/`: reusable LF reactors for sensors, motors, display, encoders, IMU, and field-level logging.

## Build

From the repository root:

```bash
lfc pololu-3pi/src/BumpMaze_ENCODER_buttons.lf
lfc pololu-3pi/src/TrackFollowSolution_encoder_imu.lf
```

or from `pololu-3pi/`:

```bash
lfc src/BumpMaze_ENCODER_buttons.lf
lfc src/TrackFollowSolution_encoder_imu.lf
```

## Operational Notes

- The application-level reactor owns sample timing, record/clear/dump control, and final CSV row construction.
- Field-level logging reactors in `lib/` pass logged values to the application logger.
- Each sample should produce one complete CSV row.
- Save captured CSV logs under `data/pololu-3pi/raw-logs/`.
