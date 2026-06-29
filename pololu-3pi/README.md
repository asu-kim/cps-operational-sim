# Pololu 3Pi+ 2040 Lingua Franca Programs

## Overview

This directory contains the Lingua Franca (LF) programs used to collect operational data from the Pololu 3Pi+ 2040 robot. The programs run on the RP2040 board, drive the robot through either a bump-maze or track-following course, and print CSV logs over USB serial. Those CSV logs are then used for plotting, error analysis, and MuJoCo replay.

The main robot programs are:

- `src/BumpMaze_ENCODER_buttons.lf`: bump-maze exploration with button-controlled logging, bump sensors, encoders, IMU fields, and odometry.
- `src/BumpMaze_ENCODER.lf`: earlier bump-maze exploration program.
- `src/TrackFollowSolution_encoder.lf`: track-following program using encoder odometry and line-sensor logging.
- `src/TrackFollowSolution_encoder_imu.lf`: track-following program using encoder distance and IMU heading for replay-oriented odometry, if present in this checkout.
- `src/TrackFollowSolution_IMU.lf`: IMU-based track-following support program, if present in this checkout.

Reusable field-level logging reactors are stored in `src/lib/`. The main LF programs own the final CSV row construction so that each sample is emitted as one complete row.

## Directory Contents

```text
pololu-3pi/
  README.md
  log-pololu-bump-encoder.ps1      # Windows PowerShell serial logger for bump-maze runs
  log-pololu-encoder.ps1           # Windows PowerShell serial logger for track-following
  src/
    *.lf                           # LF applications for the Pololu robot
    lib/
      *.lf                         # reusable LF Library reactors
  pico-sdk/                        # Raspberry Pi Pico SDK submodule
```

The `.ps1` logger scripts are kept next to this README so the robot build files and the matching host-side logging scripts are documented together.

## Prerequisites

Install or make available:

- Lingua Franca compiler: `lfc`
- RP2040/Pico C/C++ build toolchain
- Raspberry Pi Pico SDK submodule
- USB cable connected to the Pololu 3Pi+ 2040 robot
- Windows PowerShell for the serial logging scripts

Initialize the Pico SDK submodule from the repository root if needed:

```bash
git submodule update --init --recursive pololu-3pi/pico-sdk
```

## Build the LF Robot Programs

From the repository root:

```bash
cd ~/cps-operational-sim

lfc pololu-3pi/src/BumpMaze_ENCODER_buttons.lf
lfc pololu-3pi/src/TrackFollowSolution_encoder_imu.lf
```

After compiling, flash the generated UF2 or binary using the normal RP2040 workflow for the Pololu 3Pi+ 2040 robot.

```bash
picotool load bin/**.elf
```

## Host-Side Serial Logging

The robot prints CSV data over USB serial. The PowerShell scripts capture that serial output and prepend host receive timestamps:

```csv
host_receive_iso,host_receive_unix_ms,<robot CSV columns...>
```

### Bump-maze logger

Use this logger with the bump-maze program:

```powershell
cd C:\path\to\cps-operational-sim\pololu-3pi

.\log-pololu-bump-encoder.ps1 -PortName COM5 -BaudRate 115200 -TestName pololu_bump_encoder_maze
```

The bump logger is intended for:

```text
src/BumpMaze_ENCODER_buttons.lf
```

### Track-following logger

Use this logger with the track-following programs:

```powershell
cd C:\path\to\cps-operational-sim\pololu-3pi

.\log-pololu-encoder.ps1 -PortName COM5 -BaudRate 115200 -TestName pololu_line_encoder_imu_track_follow
```

This logger waits for the robot CSV header beginning with `robot_time_us,` and then writes a matching host-augmented header. This avoids stale hard-coded headers.

## Button-Controlled Logging Workflow

For the button-controlled LF programs, use this sequence:

1. Start the PowerShell logger on the host computer.
2. Reset or power on the robot.
3. Wait for calibration or initialization to finish.
4. Press button `A` to start the run and begin recording.
5. Let the robot run through the maze or track.
6. Press button `C` to stop the motors and stop recording.
7. Press button `A` again to dump the buffered CSV rows over USB serial.
8. Stop the PowerShell logger after the dump finishes.

The `.ps1` scripts write timestamped CSV files. Move or copy the resulting logs into the repository data directory when you want to analyze them:

```bash
mkdir -p ~/cps-operational-sim/data/pololu-3pi/raw-logs

cp /mnt/c/location_of_saved_file/*.csv \
   ~/cps-operational-sim/data/pololu-3pi/raw-logs/
```

## Expected Log Types

### Bump-maze CSV logs

Bump-maze logs include robot time, sample number, mode, bump sensors, encoders, odometry, heading, and motor commands. These logs are used to reconstruct the robot path, estimate detected wall locations, and compare detections against the MuJoCo ground-truth maze.

MuJoCo replay and wall-error analysis are in:

```text
mujoco-pololu-replay/src/MazeTraceReplay.lf
mujoco-pololu-replay/tools/
```

### Track-following CSV logs

track-following logs include line-sensor values, encoder readings, odometry, heading, controller state, and motor commands. These logs are used to replay the square or stadium track in MuJoCo and to quantify distance and path error.

MuJoCo replay files are in:

```text
mujoco-pololu-replay/src/RacetrackTraceReplay.lf
mujoco-pololu-replay/src/RacetrackTraceReplay_square.lf
mujoco-pololu-replay/src/RacetrackTraceReplay_imu.lf
```

## Data Organization

Recommended location:

```text
data/pololu-3pi/raw-logs/       # raw CSV logs captured from the robot
```

Keep raw logs unchanged. If a script repairs a header or filters data, save the modified file with a new name.

## Notes for Updating the Logging Schema

When adding or removing logged fields:

1. Update the LF program CSV header.
2. Update the LF final logging reaction that emits the CSV row.
3. Prefer dynamic PowerShell headers when possible, especially for track-following logs.