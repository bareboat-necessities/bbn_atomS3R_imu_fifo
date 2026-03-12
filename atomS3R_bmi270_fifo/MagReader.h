#pragma once

#include <Arduino.h>
#include <SparkFun_BMI270_Arduino_Library.h>

class MagReader
{
public:
  struct Sample
  {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
    bool updated = false;
    float deltaMs = 0.0f;
  };

  bool begin(BMI270 &imu)
  {
    if (imu.enableFeature(BMI2_AUX) != BMI2_OK)
    {
      Serial.println("Failed to enable BMI270 AUX interface.");
      return false;
    }

    bmi2_sens_config auxConfig{};
    auxConfig.type = BMI2_AUX;
    auxConfig.cfg.aux.aux_en = BMI2_ENABLE;
    auxConfig.cfg.aux.manual_en = BMI2_ENABLE;
    auxConfig.cfg.aux.man_rd_burst = BMI2_AUX_RD_BURST_FRM_LEN_1;
    auxConfig.cfg.aux.aux_rd_burst = BMI2_AUX_RD_BURST_FRM_LEN_8;
    auxConfig.cfg.aux.odr = BMI2_AUX_ODR_12_5HZ;
    auxConfig.cfg.aux.i2c_device_addr = kBmm150I2cAddr;
    auxConfig.cfg.aux.read_addr = kBmm150DataXLsbReg;
    auxConfig.cfg.aux.fcu_write_en = BMI2_DISABLE;
    auxConfig.cfg.aux.offset = 0;

    if (imu.setConfig(auxConfig) != BMI2_OK)
    {
      Serial.println("Failed to configure BMI270 AUX manual mode.");
      return false;
    }

    imu.setAuxPullUps(BMI2_ASDA_PUPSEL_2K);

    if (imu.readAux(kBmm150ChipIdReg, 1) != BMI2_OK)
    {
      Serial.println("Failed to read BMM150 chip ID.");
      return false;
    }

    if (imu.data.auxData[0] != kBmm150ChipId)
    {
      Serial.printf("Unexpected BMM150 chip ID: 0x%02X\n", imu.data.auxData[0]);
      return false;
    }

    if (imu.writeAux(kBmm150PowerCtrlReg, 0x01) != BMI2_OK)
    {
      Serial.println("Failed to power up BMM150.");
      return false;
    }
    delay(10);

    if (imu.writeAux(kBmm150OpModeReg, 0x00) != BMI2_OK)
    {
      Serial.println("Failed to configure BMM150 op mode.");
      return false;
    }

    if (imu.writeAux(kBmm150XyRepReg, 0x04) != BMI2_OK || imu.writeAux(kBmm150ZRepReg, 0x0F) != BMI2_OK)
    {
      Serial.println("Failed to configure BMM150 repetition settings.");
      return false;
    }

    auxConfig.cfg.aux.manual_en = BMI2_DISABLE;
    if (imu.setConfig(auxConfig) != BMI2_OK)
    {
      Serial.println("Failed to configure BMI270 AUX automatic mode.");
      return false;
    }

    Serial.println("BMM150 initialized over BMI270 AUX at low ODR.");
    return true;
  }

  Sample readFromAux(const uint8_t auxData[BMI2_AUX_NUM_BYTES], uint32_t sampleTimestampMs)
  {
    Sample sample{};
    decodeBMM150Raw(auxData, sample.x, sample.y, sample.z);

    sample.updated = auxDataChanged(auxData);
    if (sample.updated)
    {
      if (hasLastMagTimestamp)
      {
        sample.deltaMs = static_cast<float>(sampleTimestampMs - lastMagTimestampMs);
      }

      hasLastMagTimestamp = true;
      lastMagTimestampMs = sampleTimestampMs;
      updateLastAuxData(auxData);
    }

    return sample;
  }

private:
  static constexpr uint8_t kBmm150I2cAddr = 0x10;
  static constexpr uint8_t kBmm150ChipIdReg = 0x40;
  static constexpr uint8_t kBmm150ChipId = 0x32;
  static constexpr uint8_t kBmm150PowerCtrlReg = 0x4B;
  static constexpr uint8_t kBmm150OpModeReg = 0x4C;
  static constexpr uint8_t kBmm150XyRepReg = 0x51;
  static constexpr uint8_t kBmm150ZRepReg = 0x52;
  static constexpr uint8_t kBmm150DataXLsbReg = 0x42;

  static int16_t signExtend(int16_t value, uint8_t bits)
  {
    const int16_t shift = 16 - bits;
    return (value << shift) >> shift;
  }

  static void decodeBMM150Raw(const uint8_t auxData[BMI2_AUX_NUM_BYTES], int16_t &magX, int16_t &magY, int16_t &magZ)
  {
    const int16_t rawX = (static_cast<int16_t>(auxData[1]) << 5) | (auxData[0] >> 3);
    const int16_t rawY = (static_cast<int16_t>(auxData[3]) << 5) | (auxData[2] >> 3);
    const int16_t rawZ = (static_cast<int16_t>(auxData[5]) << 7) | (auxData[4] >> 1);

    magX = signExtend(rawX, 13);
    magY = signExtend(rawY, 13);
    magZ = signExtend(rawZ, 15);
  }

  bool auxDataChanged(const uint8_t auxData[BMI2_AUX_NUM_BYTES]) const
  {
    for (uint8_t i = 0; i < BMI2_AUX_NUM_BYTES; ++i)
    {
      if (auxData[i] != lastAuxData[i])
      {
        return true;
      }
    }

    return false;
  }

  void updateLastAuxData(const uint8_t auxData[BMI2_AUX_NUM_BYTES])
  {
    for (uint8_t i = 0; i < BMI2_AUX_NUM_BYTES; ++i)
    {
      lastAuxData[i] = auxData[i];
    }
  }

  bool hasLastMagTimestamp = false;
  uint32_t lastMagTimestampMs = 0;
  uint8_t lastAuxData[BMI2_AUX_NUM_BYTES] = {0};
};
