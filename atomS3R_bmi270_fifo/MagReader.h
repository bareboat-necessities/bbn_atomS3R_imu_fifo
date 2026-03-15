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
    bool valid = false;
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

    imu.setAuxPullUps(BMI2_ASDA_PUPSEL_2K);

    uint8_t detectedAddress = 0;
    if (!probeAndConfigureAddress(imu, detectedAddress))
    {
      Serial.println("Failed to detect BMM150 on BMI270 AUX bus.");
      return false;
    }

    bmm150I2cAddr = detectedAddress;

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
    Serial.printf("BMM150 initialized over BMI270 AUX manual-read mode (addr=0x%02X).\n", bmm150I2cAddr);
    return true;
  }


  static float rawToMicroTesla(int16_t rawValue)
  {
    // BMM150 compensated output is commonly represented at ~16 LSB per uT.
    // We expose a simple bring-up conversion for log readability.
    return static_cast<float>(rawValue) * kMicroTeslaPerLsb;
  }

  Sample readLatestSample(BMI270 &imu, uint32_t sampleTimestampMs)
  {
    Sample sample = lastSample;
    sample.valid = hasLastMagSample;

    float elapsedMs = 0.0f;
    if (hasLastMagTimestamp && sampleTimestampMs >= lastMagTimestampMs)
    {
      elapsedMs = static_cast<float>(sampleTimestampMs - lastMagTimestampMs);
    }

    if (imu.readAux(kBmm150DataXLsbReg, BMI2_AUX_NUM_BYTES) != BMI2_OK)
    {
      sample.valid = false;
      sample.updated = false;
      sample.deltaMs = 0.0f;
      return sample;
    }

    int16_t magX = 0;
    int16_t magY = 0;
    int16_t magZ = 0;
    decodeBMM150Raw(imu.data.auxData, magX, magY, magZ);

    const bool changed = !hasLastMagSample || magX != lastSample.x || magY != lastSample.y || magZ != lastSample.z;
    if (!changed)
    {
      sample.valid = false;
      sample.updated = false;
      sample.deltaMs = elapsedMs;
      return sample;
    }

    sample.x = magX;
    sample.y = magY;
    sample.z = magZ;
    sample.valid = true;
    sample.updated = true;
    sample.deltaMs = elapsedMs;

    hasLastMagTimestamp = true;
    hasLastMagSample = true;
    lastMagTimestampMs = sampleTimestampMs;
    lastSample = sample;

    return sample;
  }

private:
  static constexpr uint8_t kBmm150ChipIdReg = 0x40;
  static constexpr uint8_t kBmm150ChipId = 0x32;
  static constexpr uint8_t kBmm150PowerCtrlReg = 0x4B;
  static constexpr uint8_t kBmm150OpModeReg = 0x4C;
  static constexpr uint8_t kBmm150XyRepReg = 0x51;
  static constexpr uint8_t kBmm150ZRepReg = 0x52;
  static constexpr uint8_t kBmm150DataXLsbReg = 0x42;
  static constexpr float kMicroTeslaPerLsb = 1.0f / 16.0f;
  static bool configureAux(BMI270 &imu, uint8_t auxAddress)
  {
    bmi2_sens_config auxConfig{};
    auxConfig.type = BMI2_AUX;
    auxConfig.cfg.aux.aux_en = BMI2_ENABLE;
    auxConfig.cfg.aux.manual_en = BMI2_ENABLE;
    // Bosch BMI270 API expects encoded burst-length values (0..3),
    // where BMI2_AUX_READ_LEN_3 maps to 8-byte transfers.
    auxConfig.cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    auxConfig.cfg.aux.aux_rd_burst = BMI2_AUX_READ_LEN_3;
    auxConfig.cfg.aux.odr = BMI2_AUX_ODR_100HZ;
    auxConfig.cfg.aux.i2c_device_addr = auxAddress;
    auxConfig.cfg.aux.read_addr = kBmm150DataXLsbReg;
    auxConfig.cfg.aux.fcu_write_en = BMI2_ENABLE;
    auxConfig.cfg.aux.offset = 0;

    return imu.setConfig(auxConfig) == BMI2_OK;
  }

  static bool probeAndConfigureAddress(BMI270 &imu, uint8_t &detectedAddress)
  {
    constexpr uint8_t addressCandidates[] = {
      0x10, // BMM150 7-bit default address.
      0x12  // Alternate 7-bit address when SDO is high.
    };

    for (uint8_t auxAddress : addressCandidates)
    {
      if (!configureAux(imu, auxAddress))
      {
        continue;
      }

      // BMM150 can stay in suspend after reset; power it up before probing the
      // chip-ID register on AtomS3R's BMI270 AUX bus.
      if (imu.writeAux(kBmm150PowerCtrlReg, 0x01) != BMI2_OK)
      {
        continue;
      }
      delay(3);

      if (imu.readAux(kBmm150ChipIdReg, 1) != BMI2_OK)
      {
        continue;
      }

      if (imu.data.auxData[0] == kBmm150ChipId)
      {
        detectedAddress = auxAddress;
        return true;
      }
    }

    return false;
  }

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

  bool hasLastMagTimestamp = false;
  bool hasLastMagSample = false;
  uint32_t lastMagTimestampMs = 0;
  uint8_t bmm150I2cAddr = 0;
  Sample lastSample{};
};
