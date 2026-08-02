#include "i2c_fake.h"
#include <string.h>

static bool fake_read(prg_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                       uint8_t *buf, size_t len)
{
    prg_i2c_fake_t *fake = (prg_i2c_fake_t *)bus->ctx;

    if (addr != fake->device_addr) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        buf[i] = fake->registers[(reg + i) & 0xFF];
    }
    return true;
}

static bool fake_write(prg_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                        const uint8_t *buf, size_t len)
{
    prg_i2c_fake_t *fake = (prg_i2c_fake_t *)bus->ctx;

    if (addr != fake->device_addr) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        fake->registers[(reg + i) & 0xFF] = buf[i];
    }
    return true;
}

void prg_i2c_fake_init(prg_i2c_fake_t *fake, uint8_t device_addr)
{
    memset(fake->registers, 0, sizeof(fake->registers));
    fake->device_addr = device_addr;
}

prg_i2c_bus_t prg_i2c_fake_as_bus(prg_i2c_fake_t *fake)
{
    prg_i2c_bus_t bus;
    bus.read  = fake_read;
    bus.write = fake_write;
    bus.ctx   = fake;
    return bus;
}