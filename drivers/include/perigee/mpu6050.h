#ifndef PERIGEE_MPU6050_H
#define PERIGEE_MPU6050_H

#include "perigee/i2c_bus.h"

#define PRG_MPU6050_ADDR         0x68

#define PRG_MPU6050_REG_WHO_AM_I 0x75
#define PRG_MPU6050_WHO_AM_I_VAL 0x72

#define PRG_MPU6050_REG_PWR_MGMT_1 0x6B

#define PRG_MPU6050_REG_ACCEL_DATA 0x3B
#define PRG_MPU6050_ACCEL_DATA_LEN 14   /* accel (6) + temp (2) + gyro (6) */

/* Default full-scale ranges: accel +/-2g, gyro +/-250 deg/s.
   These divisors come straight from the datasheet's sensitivity table. */
#define PRG_MPU6050_ACCEL_LSB_PER_G     16384.0
#define PRG_MPU6050_GYRO_LSB_PER_DPS    131.0

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t temp_raw;
    int16_t gyro_x, gyro_y, gyro_z;
} prg_mpu6050_raw_t;

typedef struct {
    double accel_x_g, accel_y_g, accel_z_g;
    double temp_c;
    double gyro_x_dps, gyro_y_dps, gyro_z_dps;
} prg_mpu6050_data_t;

bool prg_mpu6050_check_id(prg_i2c_bus_t *bus);
bool prg_mpu6050_wake(prg_i2c_bus_t *bus);
bool prg_mpu6050_read_raw(prg_i2c_bus_t *bus, prg_mpu6050_raw_t *out);
void prg_mpu6050_convert(const prg_mpu6050_raw_t *raw, prg_mpu6050_data_t *out);

#endif