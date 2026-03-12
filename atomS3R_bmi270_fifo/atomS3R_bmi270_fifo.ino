#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

BMI270 imu;

constexpr uint8_t BMI270_I2C_ADDR = BMI2_I2C_PRIM_ADDR; // 0x68
constexpr uint16_t FIFO_WATERMARK_FRAMES = 20;

BMI270_SensorData fifoFrames[FIFO_WATERMARK_FRAMES];

void configureFIFO()
{
  // Use the same ODR for accel and gyro so FIFO frames are aligned.
  imu.setAccelODR(BMI2_ACC_ODR_100HZ);
  imu.setGyroODR(BMI2_GYR_ODR_100HZ);

  BMI270_FIFOConfig fifoConfig;
  fifoConfig.flags = BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN;
  fifoConfig.watermark = FIFO_WATERMARK_FRAMES;
  fifoConfig.accelDownSample = BMI2_FIFO_NO_DOWN_SAMPLING;
  fifoConfig.gyroDownSample = BMI2_FIFO_NO_DOWN_SAMPLING;
  fifoConfig.accelFilter = BMI2_ENABLE;
  fifoConfig.gyroFilter = BMI2_ENABLE;
  fifoConfig.selfWakeUp = BMI2_ENABLE;

  imu.setFIFOConfig(fifoConfig);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("M5Stack AtomS3R BMI270 FIFO demo");

  Wire.begin();

  while (imu.beginI2C(BMI270_I2C_ADDR) != BMI2_OK)
  {
    Serial.println("BMI270 init failed. Retrying in 1 second...");
    delay(1000);
  }

  configureFIFO();
  Serial.println("BMI270 initialized and FIFO configured.");
}

void loop()
{
  uint16_t fifoLength = 0;
  imu.getFIFOLength(&fifoLength);

  if (fifoLength >= FIFO_WATERMARK_FRAMES)
  {
    uint16_t framesRead = FIFO_WATERMARK_FRAMES;
    imu.getFIFOData(fifoFrames, &framesRead);

    Serial.printf("Read %u FIFO frames\n", framesRead);
    for (uint16_t i = 0; i < framesRead; ++i)
    {
      Serial.printf(
        "[%u] acc[g] x=%.3f y=%.3f z=%.3f | gyro[dps] x=%.3f y=%.3f z=%.3f\n",
        i,
        fifoFrames[i].accelX,
        fifoFrames[i].accelY,
        fifoFrames[i].accelZ,
        fifoFrames[i].gyroX,
        fifoFrames[i].gyroY,
        fifoFrames[i].gyroZ
      );
    }
  }

  delay(10);
}
