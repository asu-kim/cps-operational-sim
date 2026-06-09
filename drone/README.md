# Overview
This repository contains the working implementation of a Lingua Franca (LF) drone project with two main execution modes:

1. **Drone mode** for running with live ToF sensors on hardware: [Drone](drone/README.md)
2. **Simulation mode** for replaying ToF sensor CSV files and generating RC outputs and plots: [Simulation](simulation/README.md)

The repository is organized into:
- `lib/` for shared LF reactors and Python utilities
- `drone/` for the real drone implementation
- `simulation/` for the CSV-based simulation and plotting pipeline

# Prerequisites
You need to download this repository and have the following installed:

Python 3
pip
Lingua Franca compiler

## Library Dependencies
The following dependencies are required to run this project.
  
```
pip install numpy matplotlib pandas vl53l1x

```
### Notes on Libraries
1. ```numpy```, ```matplotlib```, and ```pandas``` are used for simulation result processing and plotting.
2. ```vl53l1x``` is required for live ToF sensor access on the drone.
3. If you are only running the simulation, you may not need vl53l1x.