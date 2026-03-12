#include "ImuReader.h"

bool ImuReader::begin(TwoWire &wire, uint8_t i2cAddress)
{
  while (imu.beginI2C(i2cAddress, wire) != BMI2_OK)
  {
    Serial.println("BMI270 init failed. Retrying in 1 second...");
    delay(1000);
  }

  return true;
}

void ImuReader::configureFIFO(uint16_t watermarkFrames)
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

ImuReader::Sample ImuReader::readLatestSample()
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

void ImuReader::sensorToNED(float sensorX, float sensorY, float sensorZ, float &north, float &east, float &down)
{
  north = sensorY;
  east = sensorX;
  down = -sensorZ;
}
