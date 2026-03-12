#pragma once

#include <Arduino.h>
#include <SparkFun_BMI270_Arduino_Library.h>

class ImuReader
{
public:
  static constexpr uint8_t kDefaultI2cAddress = BMI2_I2C_PRIM_ADDR;
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

  bool begin(TwoWire &wire, uint8_t i2cAddress = kDefaultI2cAddress)
  {
    while (imu.beginI2C(i2cAddress, wire) != BMI2_OK)
    {
      Serial.println("BMI270 init failed. Retrying in 1 second...");
      delay(1000);
    }

    return true;
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
    fifoConfig.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN | BMI2_FIFO_TIME_EN;
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
      sample.imuDeltaMs = static_cast<float>(sample.timestampMs - lastImuTimestampMs);
    }
    hasLastImuTimestamp = true;
    lastImuTimestampMs = sample.timestampMs;

    sample.accelX = latest.accelX * kStandardGravityMps2;
    sample.accelY = latest.accelY * kStandardGravityMps2;
    sample.accelZ = latest.accelZ * kStandardGravityMps2;

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
  static constexpr float kStandardGravityMps2 = 9.80665f;

  BMI270 imu;
  BMI270_SensorData fifoFrames[kDefaultFifoWatermarkFrames];
  bool hasLastImuTimestamp = false;
  uint32_t lastImuTimestampMs = 0;
};
