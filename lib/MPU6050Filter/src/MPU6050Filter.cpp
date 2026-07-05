#include "MPU6050Filter.h"

MPU6050Filter::MPU6050Filter(uint8_t addr)
    : address(addr),
      last_time_ms(0),
      angle_x(0),
      angle_y(0),
      angle_x_offset(0),
      angle_y_offset(0) {}

void MPU6050Filter::begin(int sda, int scl) {
    Wire.begin(sda, scl);

    // Wake up MPU6050
    Wire.beginTransmission(address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    last_time_ms = millis();
}

uint8_t MPU6050Filter::readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom((int)address, 1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

int16_t MPU6050Filter::read16(uint8_t regHigh) {
    Wire.beginTransmission(address);
    Wire.write(regHigh);
    Wire.endTransmission(false);
    Wire.requestFrom((int)address, 2);

    int16_t hi = Wire.available() ? Wire.read() : 0;
    int16_t lo = Wire.available() ? Wire.read() : 0;

    return (hi << 8) | lo;
}

void MPU6050Filter::computeAngles() {
    unsigned long now = millis();
    float dt = (now - last_time_ms) / 1000.0;
    last_time_ms = now;

    int16_t ax = read16(0x3B);
    int16_t ay = read16(0x3D);
    int16_t az = read16(0x3F);
    int16_t gx = read16(0x43);
    int16_t gy = read16(0x45);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;

    float acc_angle_x = atan2(ay_g, az_g) * (180.0 / M_PI);
    float acc_angle_y = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * (180.0 / M_PI);

    angle_x = 0.98 * (angle_x + gy_dps * dt) + 0.02 * acc_angle_x;
    angle_y = 0.98 * (angle_y + gx_dps * dt) + 0.02 * acc_angle_y;
}

void MPU6050Filter::calibrate(int samples) {
    float sum_x = 0;
    float sum_y = 0;

    last_time_ms = millis();

    for (int i = 0; i < samples; i++) {
        update();
        sum_x += angle_x;
        sum_y += angle_y;
        delay(5);
    }

    angle_x_offset = sum_x / samples;
    angle_y_offset = sum_y / samples;
}

void MPU6050Filter::update() {
    computeAngles();
}

float MPU6050Filter::getAngleX() {
    return angle_x - angle_x_offset;
}

float MPU6050Filter::getAngleY() {
    return angle_y - angle_y_offset;
}
