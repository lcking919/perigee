#ifndef PERIGEE_BMP388_H
#define PERIGEE_BMP388_H

#include "perigee/i2c_bus.h"

#define PRG_BMP388_ADDR       0x77
#define PRG_BMP388_REG_CHIP_ID 0x00
#define PRG_BMP388_CHIP_ID_VAL 0x50

bool prg_bmp388_check_id(prg_i2c_bus_t *bus);

#endif