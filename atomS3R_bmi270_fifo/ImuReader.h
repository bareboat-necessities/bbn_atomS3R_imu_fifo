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

  bool begin(TwoWire &wire, uint8_t i2cAddress = kDefaultI2cAddress);
  void configureFIFO(uint16_t watermarkFrames = kDefaultFifoWatermarkFrames);
  Sample readLatestSample();

  static void sensorToNED(float sensorX, float sensorY, float sensorZ, float &north, float &east, float &down);

  BMI270 &sensor() { return imu; }

private:
  static constexpr float kStandardGravityMps2 = 9.80665f;

  BMI270 imu;
  BMI270_SensorData fifoFrames[kDefaultFifoWatermarkFrames];
  bool hasLastImuTimestamp = false;
  uint32_t lastImuTimestampMs = 0;
};
