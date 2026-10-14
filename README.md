# Self-Balancing Robot (work in progress)

A two-wheeled robot built around an Arduino Nano and an MPU6050 IMU. The goal is a robot that keeps itself upright using accelerometer and gyroscope data. The current stage is tilt-controlled driving, which is the first step toward the balancing controller.

## Status

- Motor driver and motor wiring tested with `firmware/motor_test`.
- Tilt-control sketches (`tilt_motor`, `tilt_drive`) are written; hardware testing is paused.
- The TB6612FNG module overheated and failed during testing. The power stage is being debugged before the next hardware revision.
- Next: balancing code (complementary filter + PID).

## Hardware

| Part | Role |
|---|---|
| Arduino Nano | Controller |
| MPU6050 | Accelerometer + gyroscope (I2C, AD0 to GND, address 0x68) |
| TB6612FNG | Dual motor driver |
| 2x TT gear motors | Drive wheels |
| 2x 18650 in series | 7.4V battery (about 6.0 to 8.4V) |
| LM2596 module | Buck converter, set to 5.0V for the logic |
| 5A glass fuse + 6A switch | On the battery positive line |

Power path: battery + -> fuse -> switch -> LM2596 IN+ and TB6612 VM. Battery - is common ground. Pin-by-pin wiring is in [docs/wiring.md](docs/wiring.md).

## Firmware

| Sketch | Purpose |
|---|---|
| `firmware/motor_test` | Spins one motor forward and backward, no sensor needed. Use it to check the driver and wiring. |
| `firmware/tilt_motor` | One motor, driven by roll angle from the MPU6050. |
| `firmware/tilt_drive` | Two motors, with soft start, dead-zone hysteresis, driver standby, and sensor recovery. |

Each sketch sits in its own folder so the Arduino IDE can open it directly.

## Lessons so far

- Do not power the Nano from USB and the buck converter at the same time. Uploads fail.
- The MPU6050 needs a short wait after power-up before it responds.
- Motor current should not share a thin supply path with the logic; measure voltage drops across the fuse and switch while the motors run.
- A fuse rating protects wiring, not the motor driver.
