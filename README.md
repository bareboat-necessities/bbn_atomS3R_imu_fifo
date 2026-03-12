# M5Stack AtomS3R BMI270 FIFO Arduino Sketch

This repo contains an Arduino `.ino` project for reading BMI270 IMU data through the sensor FIFO on an M5Stack AtomS3R-class target.

## Project layout

- `atomS3R_bmi270_fifo/atomS3R_bmi270_fifo.ino` – configures BMI270 FIFO (accel + gyro) and prints buffered frames over serial.
- `.github/workflows/arduino-build.yml` – GitHub Actions workflow that installs Arduino CLI, the ESP32 core, SparkFun BMI270 library, and compiles the sketch.

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
