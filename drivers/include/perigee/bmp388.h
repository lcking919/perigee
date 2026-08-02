#ifndef PERIGEE_BMP388_H
#define PERIGEE_BMP388_H

#include "perigee/i2c_bus.h"

#define PRG_BMP388_ADDR       0x77
#define PRG_BMP388_REG_CHIP_ID 0x00
#define PRG_BMP388_CHIP_ID_VAL 0x50

#define PRG_BMP388_REG_DATA   0x04

typedef struct {
    uint32_t raw_pressure;
    uint32_t raw_temperature;
} prg_bmp388_raw_t;

bool prg_bmp388_read_raw(prg_i2c_bus_t *bus, prg_bmp388_raw_t *out);
bool prg_bmp388_check_id(prg_i2c_bus_t *bus);

#endif