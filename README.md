# M5Stack AtomS3R BMI270 FIFO Arduino Sketch

This repo contains an Arduino `.ino` project for reading BMI270 IMU data through the sensor FIFO on an M5Stack AtomS3R-class target.

## Project layout

- `atomS3R_bmi270_fifo/atomS3R_bmi270_fifo.ino` – sketch entrypoint that wires together reusable IMU and magnetometer readers.
- `atomS3R_bmi270_fifo/ImuReader.h` – header-only reusable BMI270 FIFO setup + sample extraction class.
- `atomS3R_bmi270_fifo/MagReader.h` – header-only reusable BMM150-over-AUX setup + sample decode class.
- `.github/workflows/arduino-build.yml` – GitHub Actions workflow that installs Arduino CLI, the ESP32 core, SparkFun BMI270 library, and compiles the sketch.

## AtomS3R hardware notes

- The internal BMI270/BMM150 sensor chain on AtomS3R is reached on the internal sensor I2C bus (`SDA=GPIO45`, `SCL=GPIO0`).
- The sketch initializes `Wire` explicitly with `Wire.begin(45, 0, 400000)` and passes that bus to `imu.beginI2C(...)`.
- BMI270 initialization probes both valid BMI270 I2C addresses (`0x68` then `0x69`) with short retry windows and fails fast if neither responds, so startup errors are explicit instead of silently retrying forever.
- Acceleration is converted to SI units (`m/s^2`) before logging.
- Gyroscope is logged in degrees/second (`dps`).
- Magnetometer values are read from BMI270 FIFO AUX frames and printed in microtesla (uT) using a simple LSB-to-uT scaling for bring-up/debug timing.
- Loop pacing targets the IMU ODR (100 Hz) by sleeping only the remaining time in each 10 ms cycle, rather than a fixed `delay(5)`.

## Build target

The workflow compiles for FQBN:

- `esp32:esp32:m5stack_atoms3`

AtomS3R uses the same ESP32-S3 family as AtomS3, making this a practical CI target for AtomS3R sketches.

## Local compile (optional)

```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "SparkFun BMI270 Arduino Library"
arduino-cli compile --fqbn esp32:esp32:m5stack_atoms3 atomS3R_bmi270_fifo
```
