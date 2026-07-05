#include <Arduino.h>
#include <Wire.h>

#include <MPU6050Filter.h>

static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 13;

static const uint8_t MPU6050_ADDR_1 = 0x68;
static const uint8_t MPU6050_ADDR_2 = 0x69;
static const uint32_t REPORT_INTERVAL_MS = 20;

MPU6050Filter sensor1(MPU6050_ADDR_1);
MPU6050Filter sensor2(MPU6050_ADDR_2);

void printSensorTilt(uint8_t address, MPU6050Filter &sensor) {
  sensor.update();

  Serial.print("0x");
  if (address < 0x10) {
    Serial.print('0');
  }
  Serial.print(address, HEX);
  Serial.print(" X=");
  Serial.print(sensor.getAngleX(), 2);
  Serial.print(" Y=");
  Serial.println(sensor.getAngleY(), 2);
}

void setup() {
  Serial.begin(115200);

  sensor1.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  sensor2.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  sensor1.calibrate(300);
  sensor2.calibrate(300);
}

void loop() {
  printSensorTilt(MPU6050_ADDR_1, sensor1);
  printSensorTilt(MPU6050_ADDR_2, sensor2);

  delay(REPORT_INTERVAL_MS);
}