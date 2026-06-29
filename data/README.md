# Data

## Overview

This directory stores raw logs, processed logs, and generated maps used by the Pololu and drone workflows.

## Directory Structure

- `drone/raw-logs/`: zipped five-sensor ToF datasets such as `Data1.zip` and `Data2.zip`
- `drone/rc-out/`: generated drone RC output logs such as `rc-out.csv`
- `pololu-3pi/raw-logs/`: raw CSV logs captured from the Pololu 3Pi+ robot

## Drone Data

Each drone ToF dataset should contain five files:

```text
front.csv
left.csv
right.csv
top.csv
bottom.csv
```

Unzip raw datasets before plotting:

```bash
cd data/drone/raw-logs
unzip Data1.zip
unzip Data2.zip
```

## Pololu Data

Raw robot logs should be saved in:

```text
data/pololu-3pi/raw-logs/
```

## Additional Instructions

- Keep raw logs unchanged after capture.
- Keep each RC output paired with the ToF dataset that produced it.
