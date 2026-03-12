#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

BMI270 imu;

constexpr uint8_t BMI270_I2C_ADDR = BMI2_I2C_PRIM_ADDR; // 0x68
constexpr uint16_t FIFO_WATERMARK_FRAMES = 20;
constexpr uint8_t ATOMS3R_SENSOR_SDA_PIN = 47;
constexpr uint8_t ATOMS3R_SENSOR_SCL_PIN = 45;
constexpr uint32_t ATOMS3R_SENSOR_I2C_HZ = 400000;
constexpr uint8_t BMM150_I2C_ADDR = 0x10;
constexpr uint8_t BMM150_CHIP_ID_REG = 0x40;
constexpr uint8_t BMM150_CHIP_ID = 0x32;
constexpr uint8_t BMM150_POWER_CTRL_REG = 0x4B;
constexpr uint8_t BMM150_OP_MODE_REG = 0x4C;
constexpr uint8_t BMM150_XY_REP_REG = 0x51;
constexpr uint8_t BMM150_Z_REP_REG = 0x52;
constexpr uint8_t BMM150_DATA_X_LSB_REG = 0x42;

BMI270_SensorData fifoFrames[FIFO_WATERMARK_FRAMES];
constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;

bool hasLastImuTimestamp = false;
uint32_t lastImuTimestampMs = 0;

bool hasLastMagTimestamp = false;
uint32_t lastMagTimestampMs = 0;
uint8_t lastAuxData[BMI2_AUX_NUM_BYTES] = {0};

int16_t signExtend(int16_t value, uint8_t bits)
{
  const int16_t shift = 16 - bits;
  return (value << shift) >> shift;
}

void decodeBMM150Raw(const uint8_t auxData[BMI2_AUX_NUM_BYTES], int16_t &magX, int16_t &magY, int16_t &magZ)
{
  const int16_t rawX = (static_cast<int16_t>(auxData[1]) << 5) | (auxData[0] >> 3);
  const int16_t rawY = (static_cast<int16_t>(auxData[3]) << 5) | (auxData[2] >> 3);
  const int16_t rawZ = (static_cast<int16_t>(auxData[5]) << 7) | (auxData[4] >> 1);

  magX = signExtend(rawX, 13);
  magY = signExtend(rawY, 13);
  magZ = signExtend(rawZ, 15);
}

bool auxDataChanged(const uint8_t auxData[BMI2_AUX_NUM_BYTES])
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

void sensorToNED(float sensorX, float sensorY, float sensorZ, float &north, float &east, float &down)
{
  // Convert ENU-style body axes to NED: N=Y, E=X, D=-Z.
  north = sensorY;
  east = sensorX;
  down = -sensorZ;
}

bool configureBMM150Aux()
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
  auxConfig.cfg.aux.odr = BMI2_AUX_ODR_12_5HZ; // Lower sample rate than accel/gyro FIFO.
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

  // Put BMM150 into normal mode and configure a low output data rate.
  if (imu.writeAux(BMM150_POWER_CTRL_REG, 0x01) != BMI2_OK)
  {
    Serial.println("Failed to power up BMM150.");
    return false;
  }
  delay(10);

  // normal mode (bits[2:1] = 00), ODR = 10Hz (bits[5:3] = 000)
  if (imu.writeAux(BMM150_OP_MODE_REG, 0x00) != BMI2_OK)
  {
    Serial.println("Failed to configure BMM150 op mode.");
    return false;
  }

  // Low-power repetition settings.
  if (imu.writeAux(BMM150_XY_REP_REG, 0x04) != BMI2_OK || imu.writeAux(BMM150_Z_REP_REG, 0x0F) != BMI2_OK)
  {
    Serial.println("Failed to configure BMM150 repetition settings.");
    return false;
  }

  // Switch aux interface to automatic mode so data is sampled in the background.
  auxConfig.cfg.aux.manual_en = BMI2_DISABLE;
  if (imu.setConfig(auxConfig) != BMI2_OK)
  {
    Serial.println("Failed to configure BMI270 AUX automatic mode.");
    return false;
  }

  Serial.println("BMM150 initialized over BMI270 AUX at low ODR.");
  return true;
}

void configureFIFO()
{
  // Keep IMU at 100 Hz and use FIFO timestamps, while mag runs slower on aux bus.
  imu.setAccelODR(BMI2_ACC_ODR_100HZ);
  imu.setGyroODR(BMI2_GYR_ODR_100HZ);

  BMI270_FIFOConfig fifoConfig;
  fifoConfig.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN | BMI2_FIFO_TIME_EN;
  fifoConfig.watermark = FIFO_WATERMARK_FRAMES;
  fifoConfig.accelDownSample = BMI2_FIFO_DOWN_SAMPLE_1;
  fifoConfig.gyroDownSample = BMI2_FIFO_DOWN_SAMPLE_1;
  fifoConfig.accelFilter = BMI2_ENABLE;
  fifoConfig.gyroFilter = BMI2_ENABLE;
  fifoConfig.selfWakeUp = BMI2_ENABLE;

  imu.setFIFOConfig(fifoConfig);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("M5Stack AtomS3R BMI270 FIFO demo (+BMM150 aux)");

  Wire.begin(ATOMS3R_SENSOR_SDA_PIN, ATOMS3R_SENSOR_SCL_PIN, ATOMS3R_SENSOR_I2C_HZ);

  while (imu.beginI2C(BMI270_I2C_ADDR, Wire) != BMI2_OK)
  {
    Serial.println("BMI270 init failed. Retrying in 1 second...");
    delay(1000);
  }

  if (!configureBMM150Aux())
  {
    Serial.println("Magnetometer init failed; continuing without valid mag data.");
  }

  configureFIFO();
  Serial.println("BMI270 initialized and FIFO configured.");
}

void loop()
{
  uint16_t fifoLength = 0;
  imu.getFIFOLength(&fifoLength);

  if (fifoLength > 0)
  {
    // Drain all currently buffered data in one read (bounded by local buffer).
    uint16_t framesRead = fifoLength;
    if (framesRead > FIFO_WATERMARK_FRAMES)
    {
      framesRead = FIFO_WATERMARK_FRAMES;
    }

    imu.getFIFOData(fifoFrames, &framesRead);

    if (framesRead > 0)
    {
      const BMI270_SensorData &latest = fifoFrames[framesRead - 1];
      const uint32_t latestTsMs = latest.sensorTimeMillis;

      float imuDeltaMs = 0.0f;
      if (hasLastImuTimestamp)
      {
        imuDeltaMs = static_cast<float>(latestTsMs - lastImuTimestampMs);
      }
      hasLastImuTimestamp = true;
      lastImuTimestampMs = latestTsMs;

      int16_t magX = 0;
      int16_t magY = 0;
      int16_t magZ = 0;
      decodeBMM150Raw(latest.auxData, magX, magY, magZ);

      float magDeltaMs = 0.0f;
      bool magUpdated = auxDataChanged(latest.auxData);
      if (magUpdated)
      {
        if (hasLastMagTimestamp)
        {
          magDeltaMs = static_cast<float>(latestTsMs - lastMagTimestampMs);
        }

        hasLastMagTimestamp = true;
        lastMagTimestampMs = latestTsMs;
        updateLastAuxData(latest.auxData);
      }

      float accNorth = 0.0f;
      float accEast = 0.0f;
      float accDown = 0.0f;
      sensorToNED(
        latest.accelX * STANDARD_GRAVITY_MPS2,
        latest.accelY * STANDARD_GRAVITY_MPS2,
        latest.accelZ * STANDARD_GRAVITY_MPS2,
        accNorth,
        accEast,
        accDown);

      float gyrNorth = 0.0f;
      float gyrEast = 0.0f;
      float gyrDown = 0.0f;
      sensorToNED(latest.gyroX, latest.gyroY, latest.gyroZ, gyrNorth, gyrEast, gyrDown);

      float magNorth = 0.0f;
      float magEast = 0.0f;
      float magDown = 0.0f;
      sensorToNED(static_cast<float>(magX), static_cast<float>(magY), static_cast<float>(magZ), magNorth, magEast, magDown);

      Serial.printf(
        "FIFO batch=%u | dt_imu_ms=%.2f | dt_mag_ms=%s%.2f | acc_ned[m/s^2] N=%.3f E=%.3f D=%.3f | gyro_ned[dps] N=%.3f E=%.3f D=%.3f | mag_ned[raw] N=%.1f E=%.1f D=%.1f\n",
        framesRead,
        imuDeltaMs,
        magUpdated ? "" : "~",
        magUpdated ? magDeltaMs : 0.0f,
        accNorth,
        accEast,
        accDown,
        gyrNorth,
        gyrEast,
        gyrDown,
        magNorth,
        magEast,
        magDown
      );
    }
  }

  delay(5);
}
