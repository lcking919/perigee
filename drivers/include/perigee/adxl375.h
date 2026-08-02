#ifndef PERIGEE_ADXL375_H
#define PERIGEE_ADXL375_H

#include "perigee/i2c_bus.h"

#define PRG_ADXL375_ADDR           0x53

#define PRG_ADXL375_REG_DEVID      0x00
#define PRG_ADXL375_DEVID_VAL      0xE5

#define PRG_ADXL375_REG_POWER_CTL  0x2D
#define PRG_ADXL375_POWER_CTL_MEASURE 0x08

#define PRG_ADXL375_REG_DATA_FORMAT 0x31
#define PRG_ADXL375_DATA_FORMAT_FULL_RES_200G 0x0B

#define PRG_ADXL375_REG_DATA       0x32
#define PRG_ADXL375_DATA_LEN       6

/* Confirmed by Analog Devices engineering forum, correcting a datasheet
   typo (0.196 g/LSB was wrong): true sensitivity is 49 mg/LSB. */
#define PRG_ADXL375_MG_PER_LSB     49.0

typedef struct {
    int16_t x, y, z;
} prg_adxl375_raw_t;

typedef struct {
    double x_g, y_g, z_g;
} prg_adxl375_data_t;

bool prg_adxl375_check_id(prg_i2c_bus_t *bus);
bool prg_adxl375_enable(prg_i2c_bus_t *bus);
bool prg_adxl375_read_raw(prg_i2c_bus_t *bus, prg_adxl375_raw_t *out);
void prg_adxl375_convert(const prg_adxl375_raw_t *raw, prg_adxl375_data_t *out);

#endif