#include <Wire.h>

#include "ImuReader.h"
#include "MagReader.h"

constexpr uint8_t ATOMS3R_SENSOR_SDA_PIN = 47;
constexpr uint8_t ATOMS3R_SENSOR_SCL_PIN = 45;
constexpr uint32_t ATOMS3R_SENSOR_I2C_HZ = 400000;

ImuReader imuReader;
MagReader magReader;

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("M5Stack AtomS3R BMI270 FIFO demo (+BMM150 aux)");

  Wire.begin(ATOMS3R_SENSOR_SDA_PIN, ATOMS3R_SENSOR_SCL_PIN, ATOMS3R_SENSOR_I2C_HZ);

  imuReader.begin(Wire);

  if (!magReader.begin(imuReader.sensor()))
  {
    Serial.println("Magnetometer init failed; continuing without valid mag data.");
  }

  imuReader.configureFIFO();
  Serial.println("BMI270 initialized and FIFO configured.");
}

void loop()
{
  const ImuReader::Sample imuSample = imuReader.readLatestSample();
  if (!imuSample.valid)
  {
    delay(5);
    return;
  }

  const MagReader::Sample magSample = magReader.readFromAux(imuSample.auxData, imuSample.timestampMs);

  float accNorth = 0.0f;
  float accEast = 0.0f;
  float accDown = 0.0f;
  ImuReader::sensorToNED(imuSample.accelX, imuSample.accelY, imuSample.accelZ, accNorth, accEast, accDown);

  float gyrNorth = 0.0f;
  float gyrEast = 0.0f;
  float gyrDown = 0.0f;
  ImuReader::sensorToNED(imuSample.gyroX, imuSample.gyroY, imuSample.gyroZ, gyrNorth, gyrEast, gyrDown);

  float magNorth = 0.0f;
  float magEast = 0.0f;
  float magDown = 0.0f;
  ImuReader::sensorToNED(static_cast<float>(magSample.x), static_cast<float>(magSample.y), static_cast<float>(magSample.z), magNorth, magEast, magDown);

  Serial.printf(
    "FIFO batch=%u | dt_imu_ms=%.2f | dt_mag_ms=%s%.2f | acc_ned[m/s^2] N=%.3f E=%.3f D=%.3f | gyro_ned[dps] N=%.3f E=%.3f D=%.3f | mag_ned[raw] N=%.1f E=%.1f D=%.1f\n",
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
    magNorth,
    magEast,
    magDown);

  delay(5);
}
