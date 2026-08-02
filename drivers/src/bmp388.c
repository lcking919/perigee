#include "perigee/bmp388.h"

bool prg_bmp388_check_id(prg_i2c_bus_t *bus)
{
    uint8_t id = 0;

    if (!bus->read(bus, PRG_BMP388_ADDR, PRG_BMP388_REG_CHIP_ID, &id, 1)) {
        return false;
    }

    return (id == PRG_BMP388_CHIP_ID_VAL);
}

bool prg_bmp388_read_raw(prg_i2c_bus_t *bus, prg_bmp388_raw_t *out)
{
    uint8_t data[6];

    if (!bus->read(bus, PRG_BMP388_ADDR, PRG_BMP388_REG_DATA, data, 6)) {
        return false;
    }

    out->raw_pressure = (uint32_t)data[0]
                      | ((uint32_t)data[1] << 8)
                      | ((uint32_t)data[2] << 16);

    out->raw_temperature = (uint32_t)data[3]
                         | ((uint32_t)data[4] << 8)
                         | ((uint32_t)data[5] << 16);

    return true;
}

static uint16_t concat_bytes(uint8_t msb, uint8_t lsb)
{
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
}

bool prg_bmp388_read_calib(prg_i2c_bus_t *bus, prg_bmp388_calib_t *calib)
{
    uint8_t d[PRG_BMP388_CALIB_LEN];

    if (!bus->read(bus, PRG_BMP388_ADDR, PRG_BMP388_REG_CALIB, d, PRG_BMP388_CALIB_LEN)) {
        return false;
    }

    uint16_t raw_t1 = concat_bytes(d[1], d[0]);
    uint16_t raw_t2 = concat_bytes(d[3], d[2]);
    int8_t   raw_t3 = (int8_t)d[4];

    int16_t raw_p1  = (int16_t)concat_bytes(d[6], d[5]);
    int16_t raw_p2  = (int16_t)concat_bytes(d[8], d[7]);
    int8_t  raw_p3  = (int8_t)d[9];
    int8_t  raw_p4  = (int8_t)d[10];
    uint16_t raw_p5 = concat_bytes(d[12], d[11]);
    uint16_t raw_p6 = concat_bytes(d[14], d[13]);
    int8_t  raw_p7  = (int8_t)d[15];
    int8_t  raw_p8  = (int8_t)d[16];
    int16_t raw_p9  = (int16_t)concat_bytes(d[18], d[17]);
    int8_t  raw_p10 = (int8_t)d[19];
    int8_t  raw_p11 = (int8_t)d[20];

    calib->par_t1 = (double)raw_t1 / 0.00390625;
    calib->par_t2 = (double)raw_t2 / 1073741824.0;
    calib->par_t3 = (double)raw_t3 / 281474976710656.0;

    calib->par_p1  = (double)(raw_p1 - 16384) / 1048576.0;
    calib->par_p2  = (double)(raw_p2 - 16384) / 536870912.0;
    calib->par_p3  = (double)raw_p3 / 4294967296.0;
    calib->par_p4  = (double)raw_p4 / 137438953472.0;
    calib->par_p5  = (double)raw_p5 / 0.125;
    calib->par_p6  = (double)raw_p6 / 64.0;
    calib->par_p7  = (double)raw_p7 / 256.0;
    calib->par_p8  = (double)raw_p8 / 32768.0;
    calib->par_p9  = (double)raw_p9 / 281474976710656.0;
    calib->par_p10 = (double)raw_p10 / 281474976710656.0;
    calib->par_p11 = (double)raw_p11 / 36893488147419103232.0;

    calib->t_lin = 0.0;

    return true;
}

double prg_bmp388_compensate_temperature(uint32_t raw_temp, prg_bmp388_calib_t *calib)
{
    double partial_data1 = (double)raw_temp - calib->par_t1;
    double partial_data2 = partial_data1 * calib->par_t2;

    calib->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib->par_t3;

    if (calib->t_lin < PRG_BMP388_MIN_TEMP_C) {
        calib->t_lin = PRG_BMP388_MIN_TEMP_C;
    }
    if (calib->t_lin > PRG_BMP388_MAX_TEMP_C) {
        calib->t_lin = PRG_BMP388_MAX_TEMP_C;
    }

    return calib->t_lin;
}

static double bmp388_pow(double base, uint8_t power)
{
    double result = 1.0;
    for (uint8_t i = 0; i < power; i++) {
        result *= base;
    }
    return result;
}

double prg_bmp388_compensate_pressure(uint32_t raw_pressure, const prg_bmp388_calib_t *calib)
{
    double partial_data1, partial_data2, partial_data3, partial_data4;
    double partial_out1, partial_out2;
    double comp_press;

    partial_data1 = calib->par_p6 * calib->t_lin;
    partial_data2 = calib->par_p7 * bmp388_pow(calib->t_lin, 2);
    partial_data3 = calib->par_p8 * bmp388_pow(calib->t_lin, 3);
    partial_out1  = calib->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib->par_p2 * calib->t_lin;
    partial_data2 = calib->par_p3 * bmp388_pow(calib->t_lin, 2);
    partial_data3 = calib->par_p4 * bmp388_pow(calib->t_lin, 3);
    partial_out2  = (double)raw_pressure *
                    (calib->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = bmp388_pow((double)raw_pressure, 2);
    partial_data2 = calib->par_p9 + calib->par_p10 * calib->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + bmp388_pow((double)raw_pressure, 3) * calib->par_p11;

    comp_press = partial_out1 + partial_out2 + partial_data4;

    if (comp_press < PRG_BMP388_MIN_PRES_PA) {
        comp_press = PRG_BMP388_MIN_PRES_PA;
    }
    if (comp_press > PRG_BMP388_MAX_PRES_PA) {
        comp_press = PRG_BMP388_MAX_PRES_PA;
    }

    return comp_press;
}