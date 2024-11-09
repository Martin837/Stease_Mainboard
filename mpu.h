#ifndef __MPU_H__
#define __MPU_H__


#include <stdio.h>

void Mpu6050_init(int16_t address);

void Mpu6050_comm(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t address);

#endif