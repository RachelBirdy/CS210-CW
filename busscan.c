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

/*

*/

const int X_MAX = 4280;
const int X_MIN = 3887;

const int Y_MAX = 4275;
const int Y_MIN = 3985;

double X_SCALE_FACTOR = 1;
double Y_SCALE_FACTOR = 1;//1.397468;
double Z_SCALE_FACTOR = 1;//0.780764;

int X_OFFSET = 0;//-3828;
int Y_OFFSET = 0;//-4125;
int Z_OFFSET = 0;//-3263;;

const int SDA_PIN = 0;
const int SCL_PIN = 1;

// The default QMC5883L address is 0x0d
static uint8_t ADDRESS = 0x0D;

static uint8_t CONTROL_REG_1 = 0x09;
static uint8_t CONTROL_REG_2 = 0x0A;
static uint8_t SET_RST_REG = 0x0B;

// Output data is 16 bit integers in two's compliment, spread
// across two registers.
const uint8_t X_LSB_REG = 0x00;
const uint8_t X_MSB_REG = 0x01;
const uint8_t Y_LSB_REG = 0x02;
const uint8_t Y_MSB_REG = 0x03;
const uint8_t Z_LSB_REG = 0x04;
const uint8_t Z_MSB_REG = 0x05;

const uint8_t STATUS_REG = 0x06;

const uint8_t TOUT_LSB = 0x07;
const uint8_t TOUT_MSB = 0x08;

// Control register 1 values. Compose the desired byte with
// bitwise OR. Addition will also work, but is less tollerant
// of mistakes.
const uint8_t MODE_CONTINUOUS = 0x01;
const uint8_t MODE_STANDBY = 0x00;
const uint8_t ODR_10HZ = 0x00;
const uint8_t ODR_50HZ = 0x04;
const uint8_t ODR_100HZ = 0x08;
const uint8_t ODR_200HZ = 0x0C;
const uint8_t RNG_2G = 0x00;
const uint8_t RNG_8G = 0x10;
const uint8_t OSR_512 = 0x00;
const uint8_t OSR_256 = 0x40;
const uint8_t OSR_128 = 0x80;
const uint8_t OSR_64 = 0xC0;

// Control register 2 values
const uint8_t INT_ENB_TRUE = 0x00;
const uint8_t INT_ENB_FALSE = 0x01;
const uint8_t ROL_PNT_TRUE = 0x40;
const uint8_t ROL_PNT_FALSE = 0x00;
const uint8_t SOFT_RST = 0x80;

void qmc5883l_init() {
    uint8_t buffer[] = {CONTROL_REG_1, (MODE_CONTINUOUS | ODR_50HZ | RNG_2G | OSR_512)};
    i2c_write_blocking(i2c0, ADDRESS, buffer, 2, false);

    buffer[0] = CONTROL_REG_2;
    buffer[1] = INT_ENB_TRUE;
    i2c_write_blocking(i2c0, ADDRESS, buffer, 2, false);
    return;
}

/*
 * Read the current values of the magnetometer
 * @return - An 16 bit array of the X, Y, and Z values
 */
int16_t *read() {
    int16_t *output = malloc(sizeof(int16_t) * 3);
    uint8_t *rawBuffer = calloc(6, sizeof(uint8_t));
    i2c_write_blocking(i2c0, ADDRESS, &X_LSB_REG, 1, true);
    i2c_read_blocking(i2c0, ADDRESS, rawBuffer, 6, false);

    output[0] = (rawBuffer[1] << 8) | rawBuffer[0];
    output[1] = (rawBuffer[3] << 8) | rawBuffer[2];
    output[2] = (rawBuffer[5] << 8) | rawBuffer[4];

    return output;
}

void calculateHeading(int* headingOut, double y, double x) {
    double heading = atan2(y, x);
    if (heading < 0) {
        heading += 2 * M_PI;
    } else if (heading > 2 * M_PI) {
        heading -= 2 * M_PI;
    }
    heading *= (double)180/M_PI;
    headingOut[0] = (int) floor(heading);
    headingOut[1] = (int) round((heading - headingOut[0]) * 60);
    return;
}

int main() {
    double *xBuffer = calloc(15, sizeof(double));
    double *yBuffer = calloc(15, sizeof(double));
    int bufferNumber = 0;

    stdio_init_all();

    X_SCALE_FACTOR = 1.0;
    Y_SCALE_FACTOR = (double) (X_MAX - X_MIN)/(Y_MAX - Y_MIN);

    X_OFFSET = X_SCALE_FACTOR * ((X_MAX-X_MIN)/2 - X_MAX);
    Y_OFFSET = Y_SCALE_FACTOR * ((Y_MAX-Y_MIN)/2 - Y_MAX);

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    qmc5883l_init();
    int xMax = -10000;
    int xMin = 5000;
    int yMax = -10000;
    int yMin = 5000;
    int zMax = -10000;
    int zMin = 5000;
    double x;
    double y;
    double z;
    int16_t *magnitudes;
    read();
    sleep_ms(1000);
    while (1) {
        magnitudes = read();
        xBuffer[bufferNumber] = magnitudes[0] * X_SCALE_FACTOR + X_OFFSET;
        yBuffer[bufferNumber] = magnitudes[1] * Y_SCALE_FACTOR + Y_OFFSET;
        z = magnitudes[2] * Z_SCALE_FACTOR + Z_OFFSET;
//        xMax = (x > xMax) ? x : xMax;
//        xMin = (x < xMin) ? x : xMin;
//        yMax = (y > yMax) ? y : yMax;
//        yMin = (y < yMin) ? y : yMin;
//        zMax = (z > zMax) ? z : zMax;
//        zMin = (z < zMin) ? z : zMin;
        x = 0;
        y = 0;
        for (int i = 0; i < 15; i++) {
            x += xBuffer[i];
            y += yBuffer[i];
        }
        y /= 15;
        x /= 15;
        //printf("x:%f,xMax:%d,xMin:%d,y:%f,yMax:%d,yMin:%d\n",x,xMax,xMin,y,yMax,yMin);
        int* heading = malloc(sizeof(int) * 2);
        calculateHeading(heading, y, x);
        bufferNumber = (bufferNumber == 15) ? 0 : (bufferNumber + 1);
        printf("Heading: %d°\n", heading[0]);
        free(magnitudes);
        free(heading);
        sleep_ms(50);
    }
    return 0;
}
