#ifndef PERIGEE_I2C_BUS_H
#define PERIGEE_I2C_BUS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef struct prg_i2c_bus prg_i2c_bus_t;

struct prg_i2c_bus {
    bool (*read)(prg_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                 uint8_t *buf, size_t len);
    bool (*write)(prg_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                  const uint8_t *buf, size_t len);
    void *ctx;
};

#endif