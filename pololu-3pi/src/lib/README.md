# Pololu LF Reactor Library

## Overview

This directory contains reusable Lingua Franca reactors for the Pololu 3Pi+ 2040 robot.

## Contents

Sensor and actuator reactors:

- `Bump.lf`: bump sensor support
- `BumpAdjustable.lf`: bump sensor support with adjustable calibration parameters
- `Display.lf`: display support
- `Encoders.lf`: wheel encoder support
- `IMU.lf`: IMU and gyro-angle support
- `Line.lf`: line sensor support
- `Motors.lf`: motor output support
- `MotorsWithFeedback.lf`: motor control with feedback support

Logging field reactors:

- `BumpLogFields.lf`: passes bump fields into the application logger
- `EncoderLogFields.lf`: passes encoder, delta encoder, distance, and command fields into the application logger
- `IMULogFields.lf`: passes gyro and target-turn fields into the application logger
- `LineSensorLogFields.lf`: passes five line-sensor values and line-following state fields into the application logger

## Design Notes

The logging reactors are intentionally field-level reactors. They do not write separate CSV files. Instead, the main application owns:

- sample timing
- `record`, `clear`, and `dump` control
- final CSV header construction
- final CSV row construction

This keeps each robot sample aligned to one CSV row.

## Usage

Import a library reactor from an LF application in `pololu-3pi/src/`:

```lf
import Encoders from "lib/Encoders.lf"
import EncoderLogFields from "lib/EncoderLogFields.lf"
```

Then instantiate it inside the application reactor:

```lf
encoder = new Encoders()
left_encoder_log = new EncoderLogFields()
right_encoder_log = new EncoderLogFields()
```
