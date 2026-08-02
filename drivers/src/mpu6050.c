#include "perigee/mpu6050.h"

static int16_t be16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

bool prg_mpu6050_check_id(prg_i2c_bus_t *bus)
{
    uint8_t id = 0;
    if (!bus->read(bus, PRG_MPU6050_ADDR, PRG_MPU6050_REG_WHO_AM_I, &id, 1)) {
        return false;
    }
    return (id == PRG_MPU6050_WHO_AM_I_VAL);
}

bool prg_mpu6050_wake(prg_i2c_bus_t *bus)
{
    uint8_t value = 0x00;
    return bus->write(bus, PRG_MPU6050_ADDR, PRG_MPU6050_REG_PWR_MGMT_1, &value, 1);
}

bool prg_mpu6050_read_raw(prg_i2c_bus_t *bus, prg_mpu6050_raw_t *out)
{
    uint8_t d[PRG_MPU6050_ACCEL_DATA_LEN];

    if (!bus->read(bus, PRG_MPU6050_ADDR, PRG_MPU6050_REG_ACCEL_DATA,
                    d, PRG_MPU6050_ACCEL_DATA_LEN)) {
        return false;
    }

    out->accel_x  = be16(d[0],  d[1]);
    out->accel_y  = be16(d[2],  d[3]);
    out->accel_z  = be16(d[4],  d[5]);
    out->temp_raw = be16(d[6],  d[7]);
    out->gyro_x   = be16(d[8],  d[9]);
    out->gyro_y   = be16(d[10], d[11]);
    out->gyro_z   = be16(d[12], d[13]);

    return true;
}

void prg_mpu6050_convert(const prg_mpu6050_raw_t *raw, prg_mpu6050_data_t *out)
{
    out->accel_x_g = (double)raw->accel_x / PRG_MPU6050_ACCEL_LSB_PER_G;
    out->accel_y_g = (double)raw->accel_y / PRG_MPU6050_ACCEL_LSB_PER_G;
    out->accel_z_g = (double)raw->accel_z / PRG_MPU6050_ACCEL_LSB_PER_G;

    out->temp_c = (double)raw->temp_raw / 340.0 + 36.53;

    out->gyro_x_dps = (double)raw->gyro_x / PRG_MPU6050_GYRO_LSB_PER_DPS;
    out->gyro_y_dps = (double)raw->gyro_y / PRG_MPU6050_GYRO_LSB_PER_DPS;
    out->gyro_z_dps = (double)raw->gyro_z / PRG_MPU6050_GYRO_LSB_PER_DPS;
}