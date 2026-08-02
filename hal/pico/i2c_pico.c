#include "i2c_pico.h"

static bool pico_read(prg_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                       uint8_t *buf, size_t len)
{
    prg_i2c_pico_t *pico = (prg_i2c_pico_t *)bus->ctx;

    int wrote = i2c_write_blocking(pico->i2c_port, addr, &reg, 1, true);
    if (wrote != 1) {
        return false;
    }

    int read = i2c_read_blocking(pico->i2c_port, addr, buf, len, false);
    return (read == (int)len);
}

static bool pico_write(prg_i2c_bus_t *bus, uint8_t addr, uint8_t reg,
                        const uint8_t *buf, size_t len)
{
    prg_i2c_pico_t *pico = (prg_i2c_pico_t *)bus->ctx;

    uint8_t packet[1 + len];
    packet[0] = reg;
    for (size_t i = 0; i < len; i++) {
        packet[1 + i] = buf[i];
    }

    int wrote = i2c_write_blocking(pico->i2c_port, addr, packet, 1 + len, false);
    return (wrote == (int)(1 + len));
}

void prg_i2c_pico_init(prg_i2c_pico_t *pico, i2c_inst_t *i2c_port)
{
    pico->i2c_port = i2c_port;
}

prg_i2c_bus_t prg_i2c_pico_as_bus(prg_i2c_pico_t *pico)
{
    prg_i2c_bus_t bus;
    bus.read  = pico_read;
    bus.write = pico_write;
    bus.ctx   = pico;
    return bus;
}