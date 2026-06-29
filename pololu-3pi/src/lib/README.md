# Pololu LF Reactor Library

## Overview

This directory contains reusable Lingua Franca reactors for the Pololu 3Pi+ 2040 robot.

## Sensor and Actuator Reactors

- `Bump.lf`: bump sensor support.
- `BumpAdjustable.lf`: bump sensor support with adjustable calibration parameters.
- `Display.lf`: OLED display support.
- `Encoders.lf`: wheel encoder support.
- `IMU.lf`: IMU and gyro-angle support.
- `Line.lf`: reflectance sensor support used by track-following programs.
- `Motors.lf`: motor output support.
- `MotorsWithFeedback.lf`: motor control with feedback support.

## Field-Level Logging Reactors

- `BumpLogFields.lf`: bump fields for maze-exploration logs.
- `EncoderLogFields.lf`: encoder, delta encoder, distance, and command fields.
- `IMULogFields.lf`: gyro and target-turn fields.
- `LineSensorLogFields.lf`: reflectance sensor values and track-following state fields.

## Design Notes

The logging reactors do not write separate CSV files. Instead, the main application owns:

- sample timing
- `record`, `clear`, and `dump` control
- final CSV header construction
- final CSV row construction

This keeps each robot sample aligned to one CSV row.

## Example Usage

```lf
import Encoders from "lib/Encoders.lf"
import EncoderLogFields from "lib/EncoderLogFields.lf"

encoder = new Encoders()
left_encoder_log = new EncoderLogFields()
right_encoder_log = new EncoderLogFields()
```
