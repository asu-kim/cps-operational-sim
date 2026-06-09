# Overview

This directory stores output logs and plots produced by the simulation workflow. It also contains the plotting script used to reconstruct an approximate drone path from RC outputs and recorded ToF sensor data.

## Contents

- `plot_drone_path.py`: generates a path-and-obstacle PDF from an RC log and a ToF data directory
- `rc-out.csv`, `rc-out-2.csv`, `rc-out-3.csv`: example RC output logs
- `*.pdf`: example generated plots

# Prerequisites

See the repository root [README.md](../../README.md) for the required Python dependencies.

### To Run the Code

Run the plotting script with explicit input paths:

```bash
python3 simulation/results/plot_drone_path.py simulation/results/rc-out-3.csv simulation/data
```

You can also run it with no arguments. In that case it defaults to:

- RC log: `simulation/results/rc-out.csv`
- ToF data directory: `simulation/data/`

```bash
python3 simulation/results/plot_drone_path.py
```

### Additional Instructions

- Make sure the RC log and the ToF CSV files come from the same run so that the samples line up correctly.
- The plotting script reads `front`, `left`, `right`, `top`, and `bottom` sensor CSV files from the selected data directory.
- The script writes a PDF next to the selected RC log using the pattern `<rc-log-name>-path-obstacles.pdf`.
- If you change the log filename in `simulation/src/test.lf`, pass the new path explicitly to `plot_drone_path.py`.