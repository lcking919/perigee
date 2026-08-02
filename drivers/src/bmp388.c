#include "perigee/bmp388.h"

bool prg_bmp388_check_id(prg_i2c_bus_t *bus)
{
    uint8_t id = 0;

    if (!bus->read(bus, PRG_BMP388_ADDR, PRG_BMP388_REG_CHIP_ID, &id, 1)) {
        return false;
    }

    return (id == PRG_BMP388_CHIP_ID_VAL);
}