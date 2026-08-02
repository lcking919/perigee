#ifndef PERIGEE_BMP388_H
#define PERIGEE_BMP388_H

#include "perigee/i2c_bus.h"

#define PRG_BMP388_ADDR       0x77
#define PRG_BMP388_REG_CHIP_ID 0x00
#define PRG_BMP388_CHIP_ID_VAL 0x50

#define PRG_BMP388_REG_DATA   0x04
#define PRG_BMP388_REG_CALIB   0x31
#define PRG_BMP388_CALIB_LEN   21

#define PRG_BMP388_MIN_TEMP_C   -40.0
#define PRG_BMP388_MAX_TEMP_C    85.0
#define PRG_BMP388_MIN_PRES_PA   30000.0
#define PRG_BMP388_MAX_PRES_PA   125000.0

typedef struct {
    double par_t1, par_t2, par_t3;
    double par_p1, par_p2, par_p3, par_p4, par_p5;
    double par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
    double t_lin;
} prg_bmp388_calib_t;

typedef struct {
    uint32_t raw_pressure;
    uint32_t raw_temperature;
} prg_bmp388_raw_t;

bool prg_bmp388_read_calib(prg_i2c_bus_t *bus, prg_bmp388_calib_t *calib);
bool prg_bmp388_read_raw(prg_i2c_bus_t *bus, prg_bmp388_raw_t *out);
bool prg_bmp388_check_id(prg_i2c_bus_t *bus);

double prg_bmp388_compensate_temperature(uint32_t raw_temp, prg_bmp388_calib_t *calib);
double prg_bmp388_compensate_pressure(uint32_t raw_pressure, const prg_bmp388_calib_t *calib);

#endif