#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "mpu.h"

/**
 * MPU6050 Motion Sensor Interface
 * ------------------------------
 * Handles initialization and communication with the MPU6050 
 * accelerometer/gyroscope sensor via I2C.
 */

void Mpu6050_init(int16_t address) {
    /* Initialize MPU6050 sensor
     * Parameters:
     * - address: I2C address of MPU6050 (typically 0x68)
     */
    uint8_t buf[2];
    buf[0] = 0x6B;              // Power management register
    buf[1] = 0x00;              // Wake up MPU6050
    i2c_write_blocking(i2c1, address, buf, 2, false);
  
    uint8_t buffer = 0x1C;
    i2c_write_blocking(i2c1, address, &buffer, 1, true); // envio el registro a escrbir
    buffer = 0b00000000;
    i2c_write_blocking(i2c1, address, &buffer, 1, false); // seteo el full scale range en 2g
    return;
}

void Mpu6050_comm(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t address) {
    /* Read accelerometer data from MPU6050
     * Parameters:
     * - accel_x/y/z: Pointers to store acceleration values
     * - address: I2C address of MPU6050
     */
    uint8_t buffer[8];
    buffer[0] = 0x3B;           // Start register for accel data

    i2c_write_blocking(i2c1, address, buffer, 1, true); // envio el registro a leer
    i2c_read_blocking(i2c1, address, buffer, 6, false); // leo el mpu6050

    *accel_x = (buffer[0] << 8) | buffer[1]; // guardo la informacion del eje x
    *accel_y = (buffer[2] << 8) | buffer[3]; // guardo la informacion del eje y
    *accel_z = (buffer[4] << 8) | buffer[5]; // guardo la informacion del eje z
    return;
}