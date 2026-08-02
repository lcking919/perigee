#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "i2c_pico.h"
#include "perigee/bmp388.h"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);   /* give the USB serial connection time to come up */

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    prg_i2c_pico_t pico_bus;
    prg_i2c_pico_init(&pico_bus, i2c0);
    prg_i2c_bus_t bus = prg_i2c_pico_as_bus(&pico_bus);

    printf("Perigee BMP388 driver test\n");

    if (!prg_bmp388_check_id(&bus)) {
        printf("FAIL: chip ID check failed - check wiring\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: chip ID confirmed\n");

    prg_bmp388_calib_t calib;
    if (!prg_bmp388_read_calib(&bus, &calib)) {
        printf("FAIL: could not read calibration data\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: calibration data read\n");

    if (!prg_bmp388_enable(&bus)) {
        printf("FAIL: could not enable normal power mode\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: sensor enabled\n");

    while (true) {
        prg_bmp388_raw_t raw;
        if (!prg_bmp388_read_raw(&bus, &raw)) {
            printf("read failed\n");
            sleep_ms(1000);
            continue;
        }

        double temp_c = prg_bmp388_compensate_temperature(raw.raw_temperature, &calib);
        double press_pa = prg_bmp388_compensate_pressure(raw.raw_pressure, &calib);

        printf("temp: %.2f C   pressure: %.2f Pa\n", temp_c, press_pa);

        sleep_ms(500);
    }
}