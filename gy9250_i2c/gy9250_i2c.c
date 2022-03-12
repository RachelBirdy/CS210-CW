/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

const uint8_t REG_GY_XOFFS = 19;
const uint8_t REG_GY_YOFFS = 21;
const uint8_t REG_GY_ZOFFS = 23;

const uint8_t REG_SAMPLE_RATE_DIV = 25;

const uint8_t REG_CONFIG = 26;
const uint8_t REG_GYRO_CONFIG = 27;
const uint8_t REG_ACCEL_CONFIG = 28;
const uint8_t REG_ACCEL_CONFIG_2 = 29;
const uint8_t REG_ACCEL_LP_ODR = 30;
const uint8_t REG_WAKE_ON_MOTION = 31;
const uint8_t REG_FIFO_ENABLE = 35;

// Reg 36 to 42 aren't important. They are for when the
// GY-9250 is acting as an i2c master for addition sensors
const uint8_t REG_I2C_MASTER = 36;
const uint8_t REG_I2C_SLV0_DEVADDR = 37;
const uint8_t REG_I2C_SLV0_REGADDR = 38;
const uint8_t REG_I2C_SLV0_CTRL = 39;
const uint8_t REG_I2C_SLV1_DEVADDR = 40;
const uint8_t REG_I2C_SLV1_REGADDR = 41;
const uint8_t REG_I2C_SLV1_CTRL = 42;

const uint8_t REG_I2C_SLV4_ADDR = 49;
const uint8_t REG_I2C_SLV4_REG = 50;
const uint8_t REG_I2C_SLV4_DO = 51;
const uint8_t REG_I2C_SLV4_CTRL = 52;
const uint8_t REG_I2C_SLV4_DI = 53;
const uint8_t REG_I2C_MST_STAT = 54;

const uint8_t REG_INT_CONFIG = 55;
const uint8_t REG_INT_ENABLE = 56;
const uint8_t REG_INT_STATUS = 58;

// Accelerometer Measurements
const uint8_t REG_ACCEL_X_H = 59;
const uint8_t REG_ACCEL_X_L = 60;
const uint8_t REG_ACCEL_Y_H = 61;
const uint8_t REG_ACCEL_Y_L = 62;
const uint8_t REG_ACCEL_Z_H = 63;
const uint8_t REG_ACCEL_Z_L = 64;

// Temperature Measurements
const uint8_t REG_TEMP_H = 65;
const uint8_t REG_TEMP_L = 66;

// Gyroscope Measurements
const uint8_t REG_GYRO_X_H = 67;
const uint8_t REG_GYRO_X_L = 68;
const uint8_t REG_GYRO_Y_H = 69;
const uint8_t REG_GYRO_Y_L = 70;
const uint8_t REG_GYRO_Z_H = 71;
const uint8_t REG_GYRO_Z_L = 72;

// Registers 73-96 store data from external i2c devices - we dont need this

// Registers 99-103 are more i2c master stuff

// Reset
const uint8_t REG_SIG_PATH_RESET = 104;

const uint8_t REG_ACCEL_INT_CTL = 105;
const uint8_t REG_USER_CTL = 106;

// Power management registers
const uint8_t REG_PWR_MGMT_1 = 107;
const uint8_t REG_PWR_MGMT_2 = 108;

const int SDA_PIN = 0;
const int SCL_PIN = 1;
const int MAG_SDA_PIN = 2;
const int MAG_SCL_PIN = 3;

// The default GY-9250 address is 0x0d
const uint8_t ADDRESS = 0x68;
const uint8_t MAG_ADDRESS = 0x0C;

const uint8_t MAG_REG_STAT_1 = 2;
const uint8_t MAG_REG_STAT_2 = 9;
const uint8_t MAG_REG_X_L = 3;
const uint8_t MAG_REG_X_H = 4;
const uint8_t MAG_REG_Y_L = 5;
const uint8_t MAG_REG_Y_H = 6;
const uint8_t MAG_REG_Z_L = 7;
const uint8_t MAG_REG_Z_H = 8;
const uint8_t MAG_REG_CTRL = 0x0A;
const uint8_t MAG_REG_X_SENS = 0x10;
const uint8_t MAG_REG_Y_SENS = 0x11;
const uint8_t MAG_REG_Z_SENS = 0x12;

const uint8_t REG_WHO_AM_I = 117;

/**
 * Read data from a register.
 */
void read_register_raw(uint8_t registerAddress, int8_t *outputBuffer, int outputLength) {
    i2c_write_blocking(i2c0, ADDRESS, &registerAddress, 1, true);
    i2c_read_blocking(i2c0, ADDRESS, outputBuffer, outputLength, false);
    return;
}

/**
 * Write data to a register
 */
void write_register_raw(uint8_t registerAddress, int8_t *writeData, int writeLength) {
    int8_t *writeBuffer = calloc(writeLength + 1, sizeof(int8_t));
    writeBuffer[0] = registerAddress;
    for (int i = 0; i < writeLength; i++) {
        writeBuffer[i + 1] = writeData[i];
    }
    i2c_write_blocking(i2c0, ADDRESS, writeData, writeLength + 1, false);
}

/**
 * Read data from a register.
 */
void read_mag_raw(uint8_t registerAddress, int8_t *outputBuffer, int outputLength) {
    i2c_write_blocking(i2c1, MAG_ADDRESS, &registerAddress, 1, true);
    i2c_read_blocking(i2c1, MAG_ADDRESS, outputBuffer, outputLength, false);
    return;
}

/**
 * Write data to a register
 */
void write_mag_raw(uint8_t registerAddress, int8_t *writeData, int writeLength) {
    int8_t *writeBuffer = calloc(writeLength + 1, sizeof(int8_t));
    writeBuffer[0] = registerAddress;
    for (int i = 0; i < writeLength; i++) {
        writeBuffer[i + 1] = writeData[i];
    }
    i2c_write_blocking(i2c1, MAG_ADDRESS, writeData, writeLength + 1, false);
}

void gy9250_init() {

    return;
}

/**
 * Read the accelerometer output
 * @return: An array of the X, Y and Z readings
 */
int16_t *read_accel() {
    int16_t *output = calloc(3, sizeof(int16_t));
    int8_t *lowByte = calloc(1, sizeof(int8_t));
    int8_t *highByte = calloc(1, sizeof(int8_t));
    read_register_raw(REG_ACCEL_X_H, highByte, 1);
    read_register_raw(REG_ACCEL_X_L, lowByte, 1);
    output[0] = (*highByte << 8) & *lowByte;
    read_register_raw(REG_ACCEL_Y_H, highByte, 1);
    read_register_raw(REG_ACCEL_Y_L, lowByte, 1);
    output[1] = (*highByte << 8) & *lowByte;
    read_register_raw(REG_ACCEL_Z_H, highByte, 1);
    read_register_raw(REG_ACCEL_Z_L, lowByte, 1);
    output[2] = (*highByte << 8) & *lowByte;
    return output;
}

/**
 * Read the gyroscope output
 * @return: An array of the X, Y and Z readings
 */
int16_t *read_gyro() {
    int16_t *output = calloc(3, sizeof(int16_t));
    int8_t *lowByte = calloc(1, sizeof(int8_t));
    int8_t *highByte = calloc(1, sizeof(int8_t));
    read_register_raw(REG_GYRO_X_H, highByte, 1);
    read_register_raw(REG_GYRO_X_L, lowByte, 1);
    output[0] =(*highByte << 8) & *lowByte;
    read_register_raw(REG_GYRO_X_H, highByte, 1);
    read_register_raw(REG_GYRO_X_L, lowByte, 1);
    output[1] =(*highByte << 8) & *lowByte;
    read_register_raw(REG_GYRO_X_H, highByte, 1);
    read_register_raw(REG_GYRO_X_L, lowByte, 1);
    output[2] =(*highByte << 8) & *lowByte;
}

int16_t *read_mag() {
    int16_t *output = calloc(3, sizeof(int16_t));
    int8_t *lowByte = calloc(1, sizeof(int8_t));
    int8_t *highByte = calloc(1, sizeof(int8_t));
//    read_mag_raw();
}

/**
 * Read the whoami register
 */
void read_who_am_i(int8_t *output) {
    i2c_write_blocking(i2c0, ADDRESS, &REG_WHO_AM_I, 1, true);
    i2c_read_blocking(i2c0, ADDRESS, output, 1, false);
}

int main() {

    stdio_init_all();

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c0, 400 * 1000);
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    while(true) {
        int8_t output;
        read_who_am_i(&output);
        printf("%x\n",output);
        sleep_ms(1000);
    }

}
