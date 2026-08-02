#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "pico_hal.h"

#include "i2c_pico.h"
#include "perigee/bmp388.h"
#include "perigee/mpu6050.h"
#include "perigee/adxl375.h"
#include "perigee/log.h"

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

    if (pico_mount(true) < 0) {
        printf("FAIL: could not mount filesystem\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: filesystem mounted\n");

    gpio_init(POWER_LED_PIN);
    gpio_set_dir(POWER_LED_PIN, GPIO_OUT);
    gpio_put(POWER_LED_PIN, 1);

    gpio_init(LOG_LED_PIN);
    gpio_set_dir(LOG_LED_PIN, GPIO_OUT);
    gpio_put(LOG_LED_PIN, 0);

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

    if (!prg_adxl375_check_id(&bus)) {
        printf("FAIL: ADXL375 chip ID check failed - check wiring\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: ADXL375 chip ID confirmed\n");

    if (!prg_adxl375_enable(&bus)) {
        printf("FAIL: could not enable ADXL375\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: ADXL375 enabled\n");

    void *log_handle = NULL;
    bool have_log = false;

    printf("Press the arm button to start/stop logging (GP%d)...\n", ARM_BUTTON_PIN);

    bool logging = false;
    bool button_was_pressed = false;

    while (true) {
        bool button_is_pressed = (gpio_get(ARM_BUTTON_PIN) == 0);

        if (button_is_pressed && !button_was_pressed) {
            logging = !logging;

            if (logging) {
                if (prg_log_open(&log_handle, "flight_test.bin")) {
                    have_log = true;
                    printf("ARMED - logging started, writing to flight_test.bin\n");
                } else {
                    printf("ARMED - logging started, but LOG FILE FAILED TO OPEN\n");
                    have_log = false;
                }
            } else {
                if (have_log) {
                    prg_log_close(log_handle);
                    have_log = false;
                }
                printf("STOPPED - logging paused, file closed\n");

                void *read_handle;
                if (prg_log_open_read(&read_handle, "flight_test.bin")) {
                    printf("--- reading back flight_test.bin ---\n");
                    prg_log_record_t rec;
                    int count = 0;
                    while (prg_log_read(read_handle, &rec)) {
                        printf("  [%d] t=%u  press=%.2f  accel_z=%.3f\n",
                               count, rec.t_ms, rec.alt_m, rec.accel_g);
                        count++;
                    }
                    printf("--- %d records read back ---\n", count);
                    prg_log_close(read_handle);
                } else {
                    printf("could not reopen file for reading\n");
                }
            }

            gpio_put(LOG_LED_PIN, logging ? 1 : 0);
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

            prg_adxl375_raw_t adxl_raw;
            if (!prg_adxl375_read_raw(&bus, &adxl_raw)) {
                printf("ADXL375 read failed\n");
                sleep_ms(1000);
                continue;
            }
            prg_adxl375_data_t adxl_data;
            prg_adxl375_convert(&adxl_raw, &adxl_data);

            printf("BMP388 - temp: %.2f C  pressure: %.2f Pa   |   MPU6050 - accel: %.2f %.2f %.2f g  gyro: %.1f %.1f %.1f dps   |   ADXL375 - %.2f %.2f %.2f g\n",
                   temp_c, press_pa,
                   mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g,
                   mpu_data.gyro_x_dps, mpu_data.gyro_y_dps, mpu_data.gyro_z_dps,
                   adxl_data.x_g, adxl_data.y_g, adxl_data.z_g);

            if (have_log) {
                prg_log_record_t rec;
                rec.t_ms       = to_ms_since_boot(get_absolute_time());
                rec.alt_m      = (float)press_pa;
                rec.accel_g    = (float)mpu_data.accel_z_g;
                rec.baro_valid = 1;
                rec.imu_valid  = 1;
                rec.state      = 0;
                if (!prg_log_write(log_handle, &rec)) {
                    printf("LOG WRITE FAILED\n");
                }
            }
        }

        sleep_ms(50);
    }
}