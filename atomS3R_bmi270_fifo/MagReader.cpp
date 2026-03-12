#include "MagReader.h"

namespace
{
constexpr uint8_t BMM150_I2C_ADDR = 0x10;
constexpr uint8_t BMM150_CHIP_ID_REG = 0x40;
constexpr uint8_t BMM150_CHIP_ID = 0x32;
constexpr uint8_t BMM150_POWER_CTRL_REG = 0x4B;
constexpr uint8_t BMM150_OP_MODE_REG = 0x4C;
constexpr uint8_t BMM150_XY_REP_REG = 0x51;
constexpr uint8_t BMM150_Z_REP_REG = 0x52;
constexpr uint8_t BMM150_DATA_X_LSB_REG = 0x42;
} // namespace

bool MagReader::begin(BMI270 &imu)
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
  auxConfig.cfg.aux.i2c_device_addr = BMM150_I2C_ADDR;
  auxConfig.cfg.aux.read_addr = BMM150_DATA_X_LSB_REG;
  auxConfig.cfg.aux.fcu_write_en = BMI2_DISABLE;
  auxConfig.cfg.aux.offset = 0;

  if (imu.setConfig(auxConfig) != BMI2_OK)
  {
    Serial.println("Failed to configure BMI270 AUX manual mode.");
    return false;
  }

  imu.setAuxPullUps(BMI2_ASDA_PUPSEL_2K);

  if (imu.readAux(BMM150_CHIP_ID_REG, 1) != BMI2_OK)
  {
    Serial.println("Failed to read BMM150 chip ID.");
    return false;
  }

  if (imu.data.auxData[0] != BMM150_CHIP_ID)
  {
    Serial.printf("Unexpected BMM150 chip ID: 0x%02X\n", imu.data.auxData[0]);
    return false;
  }

  if (imu.writeAux(BMM150_POWER_CTRL_REG, 0x01) != BMI2_OK)
  {
    Serial.println("Failed to power up BMM150.");
    return false;
  }
  delay(10);

  if (imu.writeAux(BMM150_OP_MODE_REG, 0x00) != BMI2_OK)
  {
    Serial.println("Failed to configure BMM150 op mode.");
    return false;
  }

  if (imu.writeAux(BMM150_XY_REP_REG, 0x04) != BMI2_OK || imu.writeAux(BMM150_Z_REP_REG, 0x0F) != BMI2_OK)
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

MagReader::Sample MagReader::readFromAux(const uint8_t auxData[BMI2_AUX_NUM_BYTES], uint32_t sampleTimestampMs)
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

int16_t MagReader::signExtend(int16_t value, uint8_t bits)
{
  const int16_t shift = 16 - bits;
  return (value << shift) >> shift;
}

void MagReader::decodeBMM150Raw(const uint8_t auxData[BMI2_AUX_NUM_BYTES], int16_t &magX, int16_t &magY, int16_t &magZ)
{
  const int16_t rawX = (static_cast<int16_t>(auxData[1]) << 5) | (auxData[0] >> 3);
  const int16_t rawY = (static_cast<int16_t>(auxData[3]) << 5) | (auxData[2] >> 3);
  const int16_t rawZ = (static_cast<int16_t>(auxData[5]) << 7) | (auxData[4] >> 1);

  magX = signExtend(rawX, 13);
  magY = signExtend(rawY, 13);
  magZ = signExtend(rawZ, 15);
}

bool MagReader::auxDataChanged(const uint8_t auxData[BMI2_AUX_NUM_BYTES]) const
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

void MagReader::updateLastAuxData(const uint8_t auxData[BMI2_AUX_NUM_BYTES])
{
  for (uint8_t i = 0; i < BMI2_AUX_NUM_BYTES; ++i)
  {
    lastAuxData[i] = auxData[i];
  }
}
