#include <Wire.h>

#include "ImuReader.h"
#include "MagReader.h"

constexpr uint8_t ATOMS3R_SENSOR_SDA_PIN = 45;
constexpr uint8_t ATOMS3R_SENSOR_SCL_PIN = 0;
constexpr uint32_t ATOMS3R_SENSOR_I2C_HZ = 400000;
constexpr uint32_t kImuOdrHz = 100;
constexpr uint32_t kLoopPeriodUs = 1000000UL / kImuOdrHz;

void waitForNextLoopTick(uint32_t loopStartUs)
{
  const uint32_t elapsedUs = micros() - loopStartUs;
  if (elapsedUs < kLoopPeriodUs)
  {
    delayMicroseconds(kLoopPeriodUs - elapsedUs);
  }
}

ImuReader imuReader;
MagReader magReader;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("M5Stack AtomS3R BMI270 FIFO demo (+BMM150 aux)");

  Wire.begin(ATOMS3R_SENSOR_SDA_PIN, ATOMS3R_SENSOR_SCL_PIN, ATOMS3R_SENSOR_I2C_HZ);

  if (!imuReader.begin(Wire))
  {
    Serial.println("BMI270 init failed; halting startup.");
    while (true)
    {
      delay(1000);
    }
  }

  if (!magReader.begin(imuReader.sensor()))
  {
    Serial.println("Magnetometer init failed; continuing without valid mag data.");
  }

  imuReader.configureFIFO();
  Serial.println("BMI270 initialized and FIFO configured.");
}

void loop()
{
  const uint32_t loopStartUs = micros();

  const ImuReader::Sample imuSample = imuReader.readLatestSample();
  if (!imuSample.valid)
  {
    waitForNextLoopTick(loopStartUs);
    return;
  }

  const MagReader::Sample magSample = magReader.readLatestSample(imuSample.auxData, imuSample.timestampMs);

  float accNorth = 0.0f;
  float accEast = 0.0f;
  float accDown = 0.0f;
  ImuReader::sensorToNED(imuSample.accelX, imuSample.accelY, imuSample.accelZ, accNorth, accEast, accDown);

  float gyrNorth = 0.0f;
  float gyrEast = 0.0f;
  float gyrDown = 0.0f;
  ImuReader::sensorToNED(imuSample.gyroX, imuSample.gyroY, imuSample.gyroZ, gyrNorth, gyrEast, gyrDown);

  float magNorthUt = 0.0f;
  float magEastUt = 0.0f;
  float magDownUt = 0.0f;
  ImuReader::sensorToNED(
    MagReader::rawToMicroTesla(magSample.x),
    MagReader::rawToMicroTesla(magSample.y),
    MagReader::rawToMicroTesla(magSample.z),
    magNorthUt,
    magEastUt,
    magDownUt);

  Serial.printf(
    "FIFO batch=%u | dt_imu_ms=%.2f | dt_mag_ms=%s%.2f | acc_ned[m/s^2] N=%.3f E=%.3f D=%.3f | gyro_ned[dps] N=%.3f E=%.3f D=%.3f | mag_ned[uT] N=%.2f E=%.2f D=%.2f\n",
    imuSample.framesRead,
    imuSample.imuDeltaMs,
    magSample.updated ? "" : "~",
    magSample.updated ? magSample.deltaMs : 0.0f,
    accNorth,
    accEast,
    accDown,
    gyrNorth,
    gyrEast,
    gyrDown,
    magNorthUt,
    magEastUt,
    magDownUt);

  waitForNextLoopTick(loopStartUs);
}
