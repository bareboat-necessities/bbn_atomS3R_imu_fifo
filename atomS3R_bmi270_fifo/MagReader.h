#pragma once

#include <Arduino.h>
#include <SparkFun_BMI270_Arduino_Library.h>

class MagReader
{
public:
  bool begin(BMI270 &imu);

  struct Sample
  {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    bool updated = false;
    float deltaMs = 0.0f;
  };

  Sample readFromAux(const uint8_t auxData[BMI2_AUX_NUM_BYTES], uint32_t sampleTimestampMs);

private:
  static int16_t signExtend(int16_t value, uint8_t bits);
  static void decodeBMM150Raw(const uint8_t auxData[BMI2_AUX_NUM_BYTES], int16_t &magX, int16_t &magY, int16_t &magZ);
  bool auxDataChanged(const uint8_t auxData[BMI2_AUX_NUM_BYTES]) const;
  void updateLastAuxData(const uint8_t auxData[BMI2_AUX_NUM_BYTES]);

  bool hasLastMagTimestamp = false;
  uint32_t lastMagTimestampMs = 0;
  uint8_t lastAuxData[BMI2_AUX_NUM_BYTES] = {0};
};
