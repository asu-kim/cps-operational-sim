# Pololu LF Source Programs

## Overview

This directory contains the LF application programs for the Pololu 3Pi+ 2040 robot.

## Main Files

- `BumpMaze_ENCODER_buttons.lf`: main bump-maze experiment with button control, encoder odometry, IMU heading, and CSV logging
- `TrackFollowSolution_encoder_imu.lf`: line-following experiment with encoder odometry, IMU support and line-sensor logging
- `lib/`: reusable LF reactors for sensors, motors, display, encoders, IMU, and logging fields

## To Build

From `pololu-3pi/`:

```bash
lfc src/BumpMaze_ENCODER_buttons.lf
lfc src/TrackFollowSolution_encoder_imu.lf
```

## Operational Notes

- `BumpMaze_ENCODER_buttons.lf` uses robot buttons to start, stop, and dump logs.
- `TrackFollowSolution_encoder_imu.lf` estimates `x_m`, `y_m`, and `theta_deg` using encoder odometry and IMU.
- The logging-field reactors in `lib/` are intentionally small and reusable.
- Final CSV rows are assembled in the application-level reactor so each sample produces one row.

## Output Logs

Save robot CSV logs under:

```text
../../data/pololu-3pi/raw-logs/
```
