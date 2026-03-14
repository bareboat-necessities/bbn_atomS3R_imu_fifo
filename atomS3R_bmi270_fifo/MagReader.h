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
    imuSensor = &imu;
    if (imu.enableFeature(BMI2_AUX) != BMI2_OK)
    {
      Serial.println("Failed to enable BMI270 AUX interface.");
      return false;
    }

    bmi2_sens_config auxConfig{};
    auxConfig.type = BMI2_AUX;
    auxConfig.cfg.aux.aux_en = BMI2_ENABLE;
    // FIFO AUX frames are only populated when the BMI270 AUX interface runs
    // in automatic mode. Manual mode is useful for direct readAux()/writeAux()
    // transactions but leaves FIFO aux payloads stale.
    auxConfig.cfg.aux.manual_en = BMI2_DISABLE;
    // Read a full BMM150 data frame in one automatic transaction.
    auxConfig.cfg.aux.man_rd_burst = BMI2_AUX_RD_BURST_FRM_LEN_8;
    auxConfig.cfg.aux.aux_rd_burst = BMI2_AUX_RD_BURST_FRM_LEN_8;
    auxConfig.cfg.aux.odr = BMI2_AUX_ODR_12_5HZ;
    auxConfig.cfg.aux.i2c_device_addr = kBmm150I2cAddr;
    auxConfig.cfg.aux.read_addr = kBmm150DataXLsbReg;
    // Allow forwarding writes to the external AUX device.
    auxConfig.cfg.aux.fcu_write_en = BMI2_ENABLE;
    auxConfig.cfg.aux.offset = 0;

    if (imu.setConfig(auxConfig) != BMI2_OK)
    {
      Serial.println("Failed to configure BMI270 AUX auto mode.");
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

    // Normal mode + 30 Hz ODR keeps fresh magnetometer samples available while
    // the IMU FIFO runs at 100 Hz.
    if (imu.writeAux(kBmm150OpModeReg, 0x38) != BMI2_OK)
    {
      Serial.println("Failed to configure BMM150 op mode.");
      return false;
    }

    if (imu.writeAux(kBmm150XyRepReg, 0x04) != BMI2_OK || imu.writeAux(kBmm150ZRepReg, 0x0F) != BMI2_OK)
    {
      Serial.println("Failed to configure BMM150 repetition settings.");
      return false;
    }

    delay(20);
    Serial.println("BMM150 initialized over BMI270 AUX auto read mode.");
    return true;
  }


  static float rawToMicroTesla(int16_t rawValue)
  {
    // BMM150 compensated output is commonly represented at ~16 LSB per uT.
    // We expose a simple bring-up conversion for log readability.
    return static_cast<float>(rawValue) * kMicroTeslaPerLsb;
  }

  Sample readFromAux(const uint8_t fifoAuxData[BMI2_AUX_NUM_BYTES], uint32_t sampleTimestampMs)
  {
    Sample sample = lastSample;

    if (imuSensor == nullptr)
    {
      sample.updated = false;
      sample.deltaMs = 0.0f;
      return sample;
    }

    int16_t magX = 0;
    int16_t magY = 0;
    int16_t magZ = 0;
    decodeBMM150Raw(fifoAuxData, magX, magY, magZ);

    if (!isDataReady(fifoAuxData))
    {
      sample.updated = false;
      sample.deltaMs = 0.0f;
      return sample;
    }

    const bool changed = !hasLastMagSample || magX != lastSample.x || magY != lastSample.y || magZ != lastSample.z;
    if (!changed)
    {
      sample.updated = false;
      sample.deltaMs = 0.0f;
      return sample;
    }

    sample.x = magX;
    sample.y = magY;
    sample.z = magZ;
    sample.updated = true;

    if (hasLastMagTimestamp)
    {
      sample.deltaMs = static_cast<float>(sampleTimestampMs - lastMagTimestampMs);
    }

    hasLastMagTimestamp = true;
    hasLastMagSample = true;
    lastMagTimestampMs = sampleTimestampMs;
    lastSample = sample;

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
  static constexpr float kMicroTeslaPerLsb = 1.0f / 16.0f;

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

  static bool isDataReady(const uint8_t auxData[BMI2_AUX_NUM_BYTES])
  {
    // BMM150 RHALL/STATUS byte: DRDY bit indicates a fresh sample is present.
    return (auxData[6] & 0x01) != 0;
  }


  BMI270 *imuSensor = nullptr;
  bool hasLastMagTimestamp = false;
  bool hasLastMagSample = false;
  uint32_t lastMagTimestampMs = 0;
  Sample lastSample{};
};
