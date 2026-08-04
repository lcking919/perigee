#ifndef PERIGEE_RAW_LOG_H
#define PERIGEE_RAW_LOG_H

#include <stdint.h>

typedef enum {
    PRG_SENSOR_BMP388  = 0,
    PRG_SENSOR_MPU6050 = 1,
    PRG_SENSOR_ADXL375 = 2,
} prg_sensor_id_t;

typedef struct {
    uint32_t t_ms;
    uint8_t  sensor_id;
    uint8_t  reserved[3];

    union {
        struct {
            uint32_t raw_pressure;
            uint32_t raw_temperature;
        } bmp388;

        struct {
            int16_t accel_x, accel_y, accel_z;
            int16_t temp_raw;
            int16_t gyro_x, gyro_y, gyro_z;
        } mpu6050;

        struct {
            int16_t x, y, z;
        } adxl375;
    } data;
} prg_raw_record_t;

#endif