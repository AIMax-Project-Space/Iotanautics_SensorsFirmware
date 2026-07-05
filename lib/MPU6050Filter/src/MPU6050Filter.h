#ifndef MPU6050_FILTER_H
#define MPU6050_FILTER_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050Filter {
public:
    MPU6050Filter(uint8_t addr = 0x68);

    void begin(int sda = 21, int scl = 22);
    void calibrate(int samples = 500);

    void update();  // calculate_filtered_angles()

    float getAngleX();
    float getAngleY();

private:
    uint8_t address;

    // Filter state
    unsigned long last_time_ms;
    float angle_x;
    float angle_y;

    // Offsets
    float angle_x_offset;
    float angle_y_offset;

    // Internal helpers
    uint8_t readRegister(uint8_t reg);
    int16_t read16(uint8_t regHigh);

    void computeAngles();
};

#endif
