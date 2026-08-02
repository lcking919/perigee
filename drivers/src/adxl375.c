#include "perigee/adxl375.h"

static int16_t le16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}

bool prg_adxl375_check_id(prg_i2c_bus_t *bus)
{
    uint8_t id = 0;
    if (!bus->read(bus, PRG_ADXL375_ADDR, PRG_ADXL375_REG_DEVID, &id, 1)) {
        return false;
    }
    return (id == PRG_ADXL375_DEVID_VAL);
}

bool prg_adxl375_enable(prg_i2c_bus_t *bus)
{
    uint8_t format = PRG_ADXL375_DATA_FORMAT_FULL_RES_200G;
    if (!bus->write(bus, PRG_ADXL375_ADDR, PRG_ADXL375_REG_DATA_FORMAT, &format, 1)) {
        return false;
    }

    uint8_t power = PRG_ADXL375_POWER_CTL_MEASURE;
    return bus->write(bus, PRG_ADXL375_ADDR, PRG_ADXL375_REG_POWER_CTL, &power, 1);
}

bool prg_adxl375_read_raw(prg_i2c_bus_t *bus, prg_adxl375_raw_t *out)
{
    uint8_t d[PRG_ADXL375_DATA_LEN];

    if (!bus->read(bus, PRG_ADXL375_ADDR, PRG_ADXL375_REG_DATA,
                    d, PRG_ADXL375_DATA_LEN)) {
        return false;
    }

    out->x = le16(d[0], d[1]);
    out->y = le16(d[2], d[3]);
    out->z = le16(d[4], d[5]);

    return true;
}

void prg_adxl375_convert(const prg_adxl375_raw_t *raw, prg_adxl375_data_t *out)
{
    out->x_g = (double)raw->x * (PRG_ADXL375_MG_PER_LSB / 1000.0);
    out->y_g = (double)raw->y * (PRG_ADXL375_MG_PER_LSB / 1000.0);
    out->z_g = (double)raw->z * (PRG_ADXL375_MG_PER_LSB / 1000.0);
}