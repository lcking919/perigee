#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "ff.h"

#include "i2c_pico.h"
#include "perigee/bmp388.h"
#include "perigee/mpu6050.h"
#include "perigee/adxl375.h"
#include "perigee/log.h"
#include "perigee/raw_log.h"
#include "perigee/state.h"

#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

#define ARM_BUTTON_PIN 8
#define POWER_LED_PIN  14
#define LOG_LED_PIN    15
#define LONG_PRESS_MS  3000

DWORD get_fattime(void)
{
    return ((DWORD)(2026 - 1980) << 25)
         | ((DWORD)1 << 21)
         | ((DWORD)1 << 16)
         | ((DWORD)0 << 11)
         | ((DWORD)0 << 5)
         | ((DWORD)0 >> 1);
}

static double compute_altitude_m(double current_pressure_pa, double reference_pressure_pa)
{
    return 44330.0 * (1.0 - pow(current_pressure_pa / reference_pressure_pa, 0.1903));
}

static const char *flight_state_name(prg_flight_state_t s)
{
    switch (s) {
    case PRG_STATE_IDLE:    return "IDLE";
    case PRG_STATE_ARMED:   return "ARMED";
    case PRG_STATE_BOOST:   return "BOOST";
    case PRG_STATE_COAST:   return "COAST";
    case PRG_STATE_DESCENT: return "DESCENT";
    case PRG_STATE_LANDED:  return "LANDED";
    default:                return "UNKNOWN";
    }
}

int main(void)
{
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("Perigee sensor driver test\n");

    FATFS fs;
    FRESULT fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) {
        printf("FAIL: could not mount SD card filesystem\n");
        while (true) { sleep_ms(1000); }
    }
    printf("PASS: SD card filesystem mounted\n");

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

    char calib_path[64];
    if (prg_log_next_path(calib_path, sizeof(calib_path))) {
        size_t len = strlen(calib_path);
        if (len > 4) {
            calib_path[len - 4] = '\0';
        }
        strcat(calib_path, "_calib.bin");

        void *calib_handle;
        if (prg_log_open(&calib_handle, calib_path) &&
            prg_log_write_blob(calib_handle, &calib, sizeof(calib))) {
            printf("PASS: calibration snapshot saved to %s\n", calib_path);
        } else {
            printf("WARNING: could not save calibration snapshot\n");
        }
        prg_log_close(calib_handle);
    }

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

    bool have_log = false;
    char current_log_path[64] = "";
    void *raw_log_handle = NULL;
    double ground_pressure_pa = 0.0;
    bool have_ground_pressure = false;

    prg_flight_t flight;
    bool flight_ready = false;

    printf("Press the arm button to start/stop logging (GP%d)...\n", ARM_BUTTON_PIN);

    bool logging = false;
    bool button_was_pressed = false;
    uint32_t press_start_ms = 0;
    bool long_press_fired = false;

    while (true) {
        bool button_is_pressed = (gpio_get(ARM_BUTTON_PIN) == 0);

        if (button_is_pressed && !button_was_pressed) {
            press_start_ms = to_ms_since_boot(get_absolute_time());
            long_press_fired = false;
        }

        if (button_is_pressed && button_was_pressed && !long_press_fired) {
            uint32_t held = to_ms_since_boot(get_absolute_time()) - press_start_ms;
            if (held >= LONG_PRESS_MS) {
                long_press_fired = true;

                printf("=== LONG PRESS: dumping current log ===\n");
                void *read_handle;
                if (current_log_path[0] != '\0' &&
                    prg_log_open_read(&read_handle, current_log_path)) {
                    printf("t_ms,pressure_pa,accel_z_g\n");
                    prg_log_record_t rec;
                    int count = 0;
                    while (prg_log_read(read_handle, &rec)) {
                        printf("%u,%.2f,%.3f\n", rec.t_ms, rec.alt_m, rec.accel_g);
                        count++;
                    }
                    prg_log_close(read_handle);
                    printf("=== END OF DUMP (%d records) ===\n", count);
                } else {
                    printf("could not open log for dump\n");
                }
            }
        }

        if (!button_is_pressed && button_was_pressed) {
            if (!long_press_fired) {
                logging = !logging;

                if (logging) {
                    double sum = 0.0;
                    int good_samples = 0;
                    for (int i = 0; i < 20; i++) {
                        prg_bmp388_raw_t ref_raw;
                        if (prg_bmp388_read_raw(&bus, &ref_raw)) {
                            prg_bmp388_compensate_temperature(ref_raw.raw_temperature, &calib);
                            sum += prg_bmp388_compensate_pressure(ref_raw.raw_pressure, &calib);
                            good_samples++;
                        }
                        sleep_ms(20);
                    }

                    if (good_samples > 0) {
                        ground_pressure_pa = sum / good_samples;
                        have_ground_pressure = true;
                        printf("Ground pressure captured (avg of %d): %.2f Pa\n", good_samples, ground_pressure_pa);
                    } else {
                        have_ground_pressure = false;
                        printf("WARNING: could not capture ground pressure\n");
                    }

                    if (prg_log_next_path(current_log_path, sizeof(current_log_path)) &&
                        prg_flight_init(&flight, current_log_path)) {
                        have_log = true;
                        flight.state = PRG_STATE_ARMED;
                        flight_ready = true;

                        char raw_path[64];
                        strcpy(raw_path, current_log_path);
                        size_t rlen = strlen(raw_path);
                        if (rlen > 4) {
                            raw_path[rlen - 4] = '\0';
                        }
                        strcat(raw_path, "_raw.bin");

                        if (!prg_log_open(&raw_log_handle, raw_path)) {
                            printf("WARNING: raw log failed to open\n");
                            raw_log_handle = NULL;
                        }

                        printf("ARMED - logging started, writing to %s\n", current_log_path);
                    } else {
                        printf("ARMED - logging started, but LOG FILE FAILED TO OPEN\n");
                        have_log = false;
                    }
                } else {
                    if (have_log) {
                        prg_log_close(flight.log_handle);
                        prg_log_close(raw_log_handle);
                        have_log = false;
                        flight_ready = false;
                    }
                    printf("STOPPED - logging paused, file closed\n");
                }

                gpio_put(LOG_LED_PIN, logging ? 1 : 0);
            }
        }

        button_was_pressed = button_is_pressed;

        if (logging) {
            prg_bmp388_raw_t raw;
            if (!prg_bmp388_read_raw(&bus, &raw)) {
                printf("BMP388 read failed\n");
                sleep_ms(1000);
                continue;
            }

            prg_raw_record_t rec_bmp;
            rec_bmp.t_ms = to_ms_since_boot(get_absolute_time());
            rec_bmp.sensor_id = PRG_SENSOR_BMP388;
            rec_bmp.data.bmp388.raw_pressure = raw.raw_pressure;
            rec_bmp.data.bmp388.raw_temperature = raw.raw_temperature;
            prg_log_write_blob(raw_log_handle, &rec_bmp, sizeof(rec_bmp));

            double temp_c   = prg_bmp388_compensate_temperature(raw.raw_temperature, &calib);
            double press_pa = prg_bmp388_compensate_pressure(raw.raw_pressure, &calib);

            double altitude_m = 0.0;
            if (have_ground_pressure) {
                altitude_m = compute_altitude_m(press_pa, ground_pressure_pa);
            }

            prg_mpu6050_raw_t mpu_raw;
            if (!prg_mpu6050_read_raw(&bus, &mpu_raw)) {
                printf("MPU6050 read failed\n");
                sleep_ms(1000);
                continue;
            }

            prg_raw_record_t rec_mpu;
            rec_mpu.t_ms = to_ms_since_boot(get_absolute_time());
            rec_mpu.sensor_id = PRG_SENSOR_MPU6050;
            rec_mpu.data.mpu6050.accel_x = mpu_raw.accel_x;
            rec_mpu.data.mpu6050.accel_y = mpu_raw.accel_y;
            rec_mpu.data.mpu6050.accel_z = mpu_raw.accel_z;
            rec_mpu.data.mpu6050.temp_raw = mpu_raw.temp_raw;
            rec_mpu.data.mpu6050.gyro_x = mpu_raw.gyro_x;
            rec_mpu.data.mpu6050.gyro_y = mpu_raw.gyro_y;
            rec_mpu.data.mpu6050.gyro_z = mpu_raw.gyro_z;
            prg_log_write_blob(raw_log_handle, &rec_mpu, sizeof(rec_mpu));

            prg_mpu6050_data_t mpu_data;
            prg_mpu6050_convert(&mpu_raw, &mpu_data);

            prg_adxl375_raw_t adxl_raw;
            if (!prg_adxl375_read_raw(&bus, &adxl_raw)) {
                printf("ADXL375 read failed\n");
                sleep_ms(1000);
                continue;
            }

            prg_raw_record_t rec_adxl;
            rec_adxl.t_ms = to_ms_since_boot(get_absolute_time());
            rec_adxl.sensor_id = PRG_SENSOR_ADXL375;
            rec_adxl.data.adxl375.x = adxl_raw.x;
            rec_adxl.data.adxl375.y = adxl_raw.y;
            rec_adxl.data.adxl375.z = adxl_raw.z;
            prg_log_write_blob(raw_log_handle, &rec_adxl, sizeof(rec_adxl));

            prg_adxl375_data_t adxl_data;
            prg_adxl375_convert(&adxl_raw, &adxl_data);

            printf("BMP388 - temp: %.2f C  pressure: %.2f Pa  alt: %.2f m   |   MPU6050 - accel: %.2f %.2f %.2f g  gyro: %.1f %.1f %.1f dps   |   ADXL375 - %.2f %.2f %.2f g\n",                   temp_c, press_pa, altitude_m,
                   mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g,
                   mpu_data.gyro_x_dps, mpu_data.gyro_y_dps, mpu_data.gyro_z_dps,
                   adxl_data.x_g, adxl_data.y_g, adxl_data.z_g);

            if (flight_ready) {
                prg_sample_t sample;
                sample.t_ms       = to_ms_since_boot(get_absolute_time());
                sample.alt_m      = (float)altitude_m;
                sample.accel_g    = (float)adxl_data.z_g;
                sample.baro_valid = have_ground_pressure;
                sample.imu_valid  = true;

                prg_flight_update(&flight, &sample);
                printf("FLIGHT STATE: %s\n", flight_state_name(flight.state));
            }
        }

        sleep_ms(50);
    }
}