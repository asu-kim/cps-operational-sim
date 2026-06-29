# Pololu 3Pi+ 2040 LF Programs

## Overview

This directory contains Lingua Franca (LF) programs for collecting structured operational data from the Pololu 3Pi+ 2040 robot.

The robot experiments are:

- **Track following** on square and stadium tracks.
- **Bump-based maze exploration** in a measured maze.

The robot prints CSV logs over USB serial. Host-side PowerShell scripts capture those logs and save them for plotting, MuJoCo replay, and error analysis.

## Main Files

```text
pololu-3pi/
  log-pololu-bump-encoder.ps1       # serial logger for maze exploration
  log-pololu-encoder.ps1            # serial logger for track following
  src/
    BumpMaze_ENCODER_buttons.lf     # bump-based maze exploration
    TrackFollowSolution_encoder_imu.lf
    lib/                            # reusable LF reactors
  robot-lib/                        # C support library for the Pololu robot
```

## Prerequisites

Install or make available:

- Lingua Franca compiler: `lfc`
- RP2040/Pico C/C++ build toolchain
- Raspberry Pi Pico SDK submodule
- USB connection to the Pololu 3Pi+ 2040 robot
- Windows PowerShell for serial logging

Initialize the Pico SDK submodule from the repository root:

```bash
git submodule update --init --recursive pololu-3pi/pico-sdk
```

## Build the Robot Programs

From the repository root:

```bash
lfc pololu-3pi/src/BumpMaze_ENCODER_buttons.lf
lfc pololu-3pi/src/TrackFollowSolution_encoder_imu.lf
```

Flash the generated RP2040 output using your normal Pololu/Pico workflow.

## Host-Side Serial Logging

The PowerShell loggers capture robot CSV output and prepend host receive timestamps:

```csv
host_receive_iso,host_receive_unix_ms,<robot CSV columns...>
```

### Maze-exploration logging

```powershell
cd C:\path\to\cps-operational-sim\pololu-3pi

.\log-pololu-bump-encoder.ps1 -PortName COM5 -BaudRate 115200 -TestName pololu_bump_encoder_maze
```

Use this logger with:

```text
src/BumpMaze_ENCODER_buttons.lf
```

### Track-following logging

```powershell
cd C:\path\to\cps-operational-sim\pololu-3pi

.\log-pololu-encoder.ps1 -PortName COM5 -BaudRate 115200 -TestName pololu_line_encoder_imu_track_follow
```

Use this logger with:

```text
src/TrackFollowSolution_encoder_imu.lf
```

## Button-Controlled Logging Workflow

1. Start the PowerShell logger.
2. Reset or power on the robot.
3. Wait for calibration or initialization to finish.
4. Press button `A` to start the run and begin recording.
5. Let the robot run through the track or maze.
6. Press button `C` to stop the motors and stop recording.
7. Press button `A` again to dump the buffered CSV rows over USB serial.
8. Stop the PowerShell logger after the dump finishes.

Move captured CSV files into:

```text
data/pololu-3pi/raw-logs/
```

Example from WSL:

```bash
mkdir -p ~/cps-operational-sim/data/pololu-3pi/raw-logs
cp /mnt/c/path/to/logs/*.csv ~/cps-operational-sim/data/pololu-3pi/raw-logs/
```

## Log Types

### Track-following logs

Track-following logs include line-sensor values, encoder readings, odometry, IMU heading, controller state, and motor commands. These logs are replayed in MuJoCo for the square and stadium tracks.

### Maze-exploration logs

Maze-exploration logs include bump events, encoder readings, odometry, IMU fields, controller mode, and motor commands. These logs are replayed in MuJoCo and used to estimate detected wall locations.

## Notes for Changing the Logging Schema

When adding or removing logged fields:

1. Update the LF CSV header.
2. Update the LF final logging reaction that emits each CSV row.
3. Keep the PowerShell logger dynamic when possible so it copies the robot header instead of using stale hard-coded columns.
4. Update downstream plotting and replay scripts that parse the CSV.
