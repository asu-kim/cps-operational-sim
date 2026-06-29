# Data

## Overview

This directory stores operational data used by the Pololu and drone workflows.

The data are organized by platform:

- `drone/`: ToF sensor datasets and RC command outputs for drone obstacle avoidance.
- `pololu-3pi/`: CSV logs captured from the Pololu 3Pi+ robot for track following and bump-based maze exploration.

## Directory Structure

```text
data/
  drone/
    raw-logs/
      Data1/          # original ToF data
      Data2/          # obstacle-injected or modified ToF data
      Data3/
      Data4/
      *.zip           # archived ToF datasets
    rc-out/
      rc-out.csv      # RC output generated from Data1 or first run
      rc-out-2.csv    # RC output generated from Data2 or second run
      rc-out-3.csv
      rc-out-4.csv
  pololu-3pi/
    raw-logs/
      *.csv           # Pololu track-following and maze-exploration logs
```

## Drone Data

Each ToF dataset directory should contain:

```text
front.csv
left.csv
right.csv
top.csv
bottom.csv
```

The matching RC output logs are stored in `data/drone/rc-out/`.

Typical pairing:

```text
data/drone/rc-out/rc-out.csv    -> data/drone/raw-logs/Data1/
data/drone/rc-out/rc-out-2.csv  -> data/drone/raw-logs/Data2/
```

## Pololu Data

Pololu CSV logs are captured over USB serial using the PowerShell scripts in `pololu-3pi/`.

Typical log types:

- track-following logs from square and stadium tracks
- bump-based maze-exploration logs
- encoder, IMU, bump, controller-state, and actuator-command fields

Keep captured logs in:

```text
data/pololu-3pi/raw-logs/
```

## Notes

- Keep raw logs unchanged.
- Save derived or repaired files with a new name.
- Keep filenames descriptive enough to identify the experiment and capture time.
