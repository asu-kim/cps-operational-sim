# Drone Shared Library

## Overview

This directory contains shared LF reactors and Python helpers for the drone obstacle-avoidance workflow.

The same planner and output logic can be used with live ToF streams or CSV-backed ToF replay.

## Contents

- `ToFBridgeC.lf`: live ToF bridge that launches `tof_reader.py`.
- `UserLandCmd.lf`: user landing command helper.
- `avoid_planner_modal.lf`: modal obstacle-avoidance planner.
- `msp_sender.lf`: MSP RC command sender or CSV logger.
- `tof_reader.py`: Python helper for one live ToF sensor stream.
- `tof_logger.py`: helper for recording ToF streams into CSV files.

## MSP Sender Modes

`msp_sender.lf` can operate in two modes:

- serial mode when `port` is a device path, such as `/dev/ttyACM0`
- log-only mode when `port=""`

The CSV simulation uses log-only mode to write RC command outputs.

## Notes

- Keep sensor sample periods consistent across ToF capture, LF replay, and plotting.
- Use repository-local LF imports when possible so the project can run on another machine.
- The five sensor names used throughout the project are `front`, `left`, `right`, `top`, and `bottom`.
