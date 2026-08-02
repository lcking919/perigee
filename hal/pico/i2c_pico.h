#ifndef PERIGEE_I2C_PICO_H
#define PERIGEE_I2C_PICO_H

#include "perigee/i2c_bus.h"
#include "hardware/i2c.h"

typedef struct {
    i2c_inst_t *i2c_port;
} prg_i2c_pico_t;

void prg_i2c_pico_init(prg_i2c_pico_t *pico, i2c_inst_t *i2c_port);
prg_i2c_bus_t prg_i2c_pico_as_bus(prg_i2c_pico_t *pico);

#endif