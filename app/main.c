#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "i2c_pico.h"
#include "perigee/bmp388.h"
#include "perigee/mpu6050.h"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define ARM_BUTTON_PIN 8
#define POWER_LED_PIN  14
#define LOG_LED_PIN    15

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("Perigee sensor driver test\n");

    gpio_init(POWER_LED_PIN);
    gpio_set_dir(POWER_LED_PIN, GPIO_OUT);
    gpio_put(POWER_LED_PIN, 1);   /* on the instant the program starts */

    gpio_init(LOG_LED_PIN);
    gpio_set_dir(LOG_LED_PIN, GPIO_OUT);
    gpio_put(LOG_LED_PIN, 0);     /* off until logging actually begins */

    gpio_init(ARM_BUTTON_PIN);
    gpio_set_dir(ARM_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(ARM_BUTTON_PIN);

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    prg_i2c_pico_t pico_bus;
    prg_i2c_pico_init(&pico_bus, i2c0);
    prg_i2c_bus_t bus = prg_i2c_pico_as_bus(&pico_bus);

    if (!prg_bmp388_check_id(&bus)) {
        printf("FAIL: BMP388 chip ID check failed - check wiring\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: BMP388 chip ID confirmed\n");

    prg_bmp388_calib_t calib;
    if (!prg_bmp388_read_calib(&bus, &calib)) {
        printf("FAIL: could not read BMP388 calibration data\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: BMP388 calibration data read\n");

    if (!prg_bmp388_enable(&bus)) {
        printf("FAIL: could not enable BMP388 normal power mode\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: BMP388 sensor enabled\n");

    if (!prg_mpu6050_check_id(&bus)) {
        printf("FAIL: MPU6050 chip ID check failed - check wiring\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: MPU6050 chip ID confirmed\n");

    if (!prg_mpu6050_wake(&bus)) {
        printf("FAIL: could not wake MPU6050\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: MPU6050 awake\n");

    printf("Press the arm button to start/stop logging (GP%d)...\n", ARM_BUTTON_PIN);

    bool logging = false;
    bool button_was_pressed = false;

    while (true) {
        bool button_is_pressed = (gpio_get(ARM_BUTTON_PIN) == 0);

        if (button_is_pressed && !button_was_pressed) {
            logging = !logging;
            gpio_put(LOG_LED_PIN, logging ? 1 : 0);
            printf(logging ? "ARMED - logging started\n" : "STOPPED - logging paused\n");
        }
        button_was_pressed = button_is_pressed;

        if (logging) {
            prg_bmp388_raw_t raw;
            if (!prg_bmp388_read_raw(&bus, &raw)) {
                printf("BMP388 read failed\n");
                sleep_ms(1000);
                continue;
            }

            double temp_c   = prg_bmp388_compensate_temperature(raw.raw_temperature, &calib);
            double press_pa = prg_bmp388_compensate_pressure(raw.raw_pressure, &calib);

            prg_mpu6050_raw_t mpu_raw;
            if (!prg_mpu6050_read_raw(&bus, &mpu_raw)) {
                printf("MPU6050 read failed\n");
                sleep_ms(1000);
                continue;
            }
            prg_mpu6050_data_t mpu_data;
            prg_mpu6050_convert(&mpu_raw, &mpu_data);

            printf("BMP388 - temp: %.2f C  pressure: %.2f Pa   |   MPU6050 - accel: %.2f %.2f %.2f g  gyro: %.1f %.1f %.1f dps\n",
                   temp_c, press_pa,
                   mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g,
                   mpu_data.gyro_x_dps, mpu_data.gyro_y_dps, mpu_data.gyro_z_dps);
        }

        sleep_ms(50);
    }
}