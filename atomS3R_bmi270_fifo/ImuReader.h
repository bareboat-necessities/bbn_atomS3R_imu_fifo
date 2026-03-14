#pragma once

#include <Arduino.h>
#include <SparkFun_BMI270_Arduino_Library.h>

class ImuReader
{
public:
  static constexpr uint8_t kDefaultI2cAddress = BMI2_I2C_PRIM_ADDR;
  static constexpr uint8_t kAlternateI2cAddress = BMI2_I2C_SEC_ADDR;
  static constexpr uint16_t kDefaultFifoWatermarkFrames = 20;

  struct Sample
  {
    bool valid = false;
    uint16_t framesRead = 0;
    uint32_t timestampMs = 0;
    float imuDeltaMs = 0.0f;
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    uint8_t auxData[BMI2_AUX_NUM_BYTES] = {0};
  };

  bool begin(TwoWire &wire, uint8_t i2cAddress = kDefaultI2cAddress, uint8_t attemptsPerAddress = 3)
  {
    uint8_t addressesToTry[2] = {i2cAddress, kAlternateI2cAddress};
    if (i2cAddress == kAlternateI2cAddress)
    {
      addressesToTry[1] = kDefaultI2cAddress;
    }

    for (uint8_t attempt = 1; attempt <= attemptsPerAddress; ++attempt)
    {
      for (const uint8_t address : addressesToTry)
      {
        if (imu.beginI2C(address, wire) == BMI2_OK)
        {
          Serial.printf(
            "BMI270 detected at I2C address 0x%02X (attempt %u/%u)\n",
            address,
            attempt,
            attemptsPerAddress);
          return true;
        }
      }

      delay(50);
    }

    Serial.printf(
      "BMI270 init failed on 0x%02X and 0x%02X after %u attempts each.\n",
      addressesToTry[0],
      addressesToTry[1],
      attemptsPerAddress);

    return false;
  }

  void configureFIFO(uint16_t watermarkFrames = kDefaultFifoWatermarkFrames)
  {
    if (watermarkFrames > kDefaultFifoWatermarkFrames)
    {
      watermarkFrames = kDefaultFifoWatermarkFrames;
    }

    imu.setAccelODR(BMI2_ACC_ODR_100HZ);
    imu.setGyroODR(BMI2_GYR_ODR_100HZ);

    BMI270_FIFOConfig fifoConfig;
    // Keep FIFO to IMU streams only. AUX reads are performed explicitly through
    // MagReader, which avoids frame parsing misalignment from mixed AUX frames.
    fifoConfig.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_TIME_EN;
    fifoConfig.watermark = watermarkFrames;
    fifoConfig.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_1;
    fifoConfig.gyroDownSample = BMI2_FIFO_DOWN_SAMPLE_1;
    fifoConfig.accelFilter = BMI2_ENABLE;
    fifoConfig.gyroFilter = BMI2_ENABLE;
    fifoConfig.selfWakeUp = BMI2_ENABLE;

    imu.setFIFOConfig(fifoConfig);
  }

  Sample readLatestSample()
  {
    Sample sample{};
    uint16_t fifoLength = 0;
    imu.getFIFOLength(&fifoLength);

    if (fifoLength == 0)
    {
      return sample;
    }

    uint16_t framesRead = fifoLength;
    if (framesRead > kDefaultFifoWatermarkFrames)
    {
      framesRead = kDefaultFifoWatermarkFrames;
    }

    imu.getFIFOData(fifoFrames, &framesRead);
    if (framesRead == 0)
    {
      return sample;
    }

    const BMI270_SensorData &latest = fifoFrames[framesRead - 1];
    sample.valid = true;
    sample.framesRead = framesRead;

    sample.timestampMs = latest.sensorTimeMillis;

    if (hasLastImuTimestamp)
    {
      uint32_t timestampDeltaMs = 0;

      if (sample.timestampMs > lastImuTimestampMs)
      {
        timestampDeltaMs = sample.timestampMs - lastImuTimestampMs;
      }

      // If FIFO sensor time is missing or non-monotonic, estimate elapsed time
      // from the number of frames drained at the configured 100 Hz ODR.
      if (timestampDeltaMs == 0)
      {
        timestampDeltaMs = framesRead * kImuFramePeriodMs;
        sample.timestampMs = lastImuTimestampMs + timestampDeltaMs;
      }

      // Report per-sample IMU period rather than per-batch elapsed time.
      // This keeps dt stable even when FIFO drains 2-3 frames at once.
      sample.imuDeltaMs = static_cast<float>(timestampDeltaMs) / static_cast<float>(framesRead);
    }

    hasLastImuTimestamp = true;
    lastImuTimestampMs = sample.timestampMs;

    // The SparkFun BMI270 library already reports acceleration in m/s^2.
    // Do not scale by g again.
    sample.accelX = latest.accelX;
    sample.accelY = latest.accelY;
    sample.accelZ = latest.accelZ;

    sample.gyroX = latest.gyroX;
    sample.gyroY = latest.gyroY;
    sample.gyroZ = latest.gyroZ;

    for (uint8_t i = 0; i < BMI2_AUX_NUM_BYTES; ++i)
    {
      sample.auxData[i] = latest.auxData[i];
    }

    return sample;
  }

  static void sensorToNED(float sensorX, float sensorY, float sensorZ, float &north, float &east, float &down)
  {
    north = sensorY;
    east = sensorX;
    down = -sensorZ;
  }

  BMI270 &sensor() { return imu; }

private:
  static constexpr uint32_t kImuFramePeriodMs = 10;

  BMI270 imu;
  BMI270_SensorData fifoFrames[kDefaultFifoWatermarkFrames];
  bool hasLastImuTimestamp = false;
  uint32_t lastImuTimestampMs = 0;
};
