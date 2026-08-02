#include "perigee/bmp388.h"

bool prg_bmp388_check_id(prg_i2c_bus_t *bus)
{
    uint8_t id = 0;

    if (!bus->read(bus, PRG_BMP388_ADDR, PRG_BMP388_REG_CHIP_ID, &id, 1)) {
        return false;
    }

    return (id == PRG_BMP388_CHIP_ID_VAL);
}

bool prg_bmp388_read_raw(prg_i2c_bus_t *bus, prg_bmp388_raw_t *out)
{
    uint8_t data[6];

    if (!bus->read(bus, PRG_BMP388_ADDR, PRG_BMP388_REG_DATA, data, 6)) {
        return false;
    }

    out->raw_pressure = (uint32_t)data[0]
                      | ((uint32_t)data[1] << 8)
                      | ((uint32_t)data[2] << 16);

    out->raw_temperature = (uint32_t)data[3]
                         | ((uint32_t)data[4] << 8)
                         | ((uint32_t)data[5] << 16);

    return true;
}