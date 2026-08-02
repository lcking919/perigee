#ifndef PERIGEE_I2C_FAKE_H
#define PERIGEE_I2C_FAKE_H

#include "perigee/i2c_bus.h"

typedef struct {
    uint8_t registers[256];
    uint8_t device_addr;
} prg_i2c_fake_t;

void prg_i2c_fake_init(prg_i2c_fake_t *fake, uint8_t device_addr);
prg_i2c_bus_t prg_i2c_fake_as_bus(prg_i2c_fake_t *fake);

#endif