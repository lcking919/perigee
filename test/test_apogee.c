#include "perigee/apogee.h"
#include "perigee/vapogee.h"
#include "perigee/boost.h"
#include "perigee/state.h"
#include "perigee/log.h"
#include "i2c_fake.h"
#include "perigee/bmp388.h"
#include "perigee/mpu6050.h"
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>


static int failures = 0;

#define CHECK(cond, msg)                                  \
    do {                                                  \
        if (cond) {                                       \
            printf("  pass  %s\n", msg);                  \
        } else {                                          \
            printf("  FAIL  %s   (%s:%d)\n",              \
                   msg, __FILE__, __LINE__);              \
            failures++;                                   \
        }                                                 \
    } while (0)

#define APEX_M   200.0f
#define APEX_MS  5000
#define DT_MS    20
#define DUR_MS   9000

static float profile_alt(uint32_t t_ms)
{
    float dt = ((float)t_ms - (float)APEX_MS) / 1000.0f;
    float a  = APEX_M - 0.5f * 9.81f * dt * dt;
    return a < 0.0f ? 0.0f : a;
}

static prg_sample_t make(uint32_t t, float alt, bool valid)
{
    prg_sample_t s;
    s.t_ms       = t;
    s.alt_m      = alt;
    s.accel_g    = 0.0f;
    s.baro_valid = valid;
    s.imu_valid  = true;
    return s;
}

static void test_detects_on_clean_profile(void)
{
    printf("test: clean ballistic profile\n");

    prg_apogee_t det;
    prg_apogee_init(&det);

    for (uint32_t t = 0; t <= DUR_MS; t += DT_MS) {
        prg_sample_t s = make(t, profile_alt(t), true);
        prg_apogee_update(&det, &s);
    }

    CHECK(det.detected, "apogee was detected");

    long lag = (long)det.detected_t_ms - APEX_MS;
    printf("        peak %u ms, declared %u ms, lag %ld ms\n",
           det.max_alt_t_ms, det.detected_t_ms, lag);

    CHECK(lag > 0, "declared after apogee, not before");
    /* This detector cannot meet PRG-FSW-030 (200ms) - see vapogee.c,
       which meets it at 60ms. Kept here as a measured baseline. */
}

static void test_no_false_positive_climbing(void)
{
    printf("test: no false positive while climbing\n");

    prg_apogee_t det;
    prg_apogee_init(&det);

    for (uint32_t t = 0; t < 4000; t += DT_MS) {
        prg_sample_t s = make(t, profile_alt(t), true);
        prg_apogee_update(&det, &s);
    }

    CHECK(!det.detected, "still climbing, nothing declared");
}

static void test_single_glitch_rejected(void)
{
    printf("test: one bad sample does not trigger\n");

    prg_apogee_t det;
    prg_apogee_init(&det);

    for (uint32_t t = 0; t < 3000; t += DT_MS) {
        float alt = profile_alt(t);
        if (t == 2000) {
            alt -= 5.0f;
        }
        prg_sample_t s = make(t, alt, true);
        prg_apogee_update(&det, &s);
    }

    CHECK(!det.detected, "single glitch rejected");
}

static void test_velocity_detector(void)
{
    printf("test: velocity detector on clean profile\n");

    prg_vapogee_t det;
    prg_vapogee_init(&det);

    for (uint32_t t = 0; t <= DUR_MS; t += DT_MS) {
        prg_sample_t s = make(t, profile_alt(t), true);
        prg_vapogee_update(&det, &s);
    }

    CHECK(det.detected, "apogee was detected");

    long lag = (long)det.detected_t_ms - APEX_MS;
    printf("        declared %u ms, lag %ld ms\n", det.detected_t_ms, lag);

    CHECK(lag > 0, "declared after apogee, not before");
    CHECK(lag <= 200, "PRG-FSW-030: within 200 ms of apogee");
}

static void test_velocity_detector_with_noise(void)
{
    printf("test: velocity detector with sensor noise\n");

    prg_vapogee_t det;
    prg_vapogee_init(&det);

    unsigned seed = 42;
    for (uint32_t t = 0; t <= DUR_MS; t += DT_MS) {
        seed = seed * 1103515245u + 12345u;
        float noise = (((float)((seed >> 16) & 0x7fff) / 16383.5f) - 1.0f) * 0.2f;
        prg_sample_t s = make(t, profile_alt(t) + noise, true);
        prg_vapogee_update(&det, &s);
    }

    if (det.detected) {
        long lag = (long)det.detected_t_ms - APEX_MS;
        printf("        declared %u ms, lag %ld ms\n", det.detected_t_ms, lag);
        CHECK(lag > -300 && lag < 800, "detected somewhere near apogee, not wildly off");
    } else {
        CHECK(0, "apogee was detected at all");
    }
}

static void test_boost_detects_ignition(void)
{
    printf("test: boost detection on a simple burn profile\n");

    prg_boost_t det;
    prg_boost_init(&det);

    /* Pad, sitting still: ~1g, well below threshold */
    for (uint32_t t = 0; t < 1000; t += DT_MS) {
        prg_sample_t s = make(t, 0.0f, true);
        s.accel_g = 1.0f;
        prg_boost_update(&det, &s);
    }

    CHECK(!det.detected, "not detected while sitting on the pad");

    /* Ignition at t=1000ms: acceleration jumps to 5g and holds */
    for (uint32_t t = 1000; t < 3000; t += DT_MS) {
        prg_sample_t s = make(t, 0.0f, true);
        s.accel_g = 5.0f;
        bool hit = prg_boost_update(&det, &s);
        if (hit && !det.detected) {
            /* unreachable - det.detected already reflects hit */
        }
    }

    CHECK(det.detected, "boost was detected during the burn");

    long delay = (long)det.detected_t_ms - 1000L;
    printf("        ignition at 1000 ms, declared %u ms, delay %ld ms\n",
           det.detected_t_ms, delay);

    CHECK(delay >= (long)PRG_BOOST_HOLD_MS,
          "declared no sooner than the required hold time");
    CHECK(delay < (long)PRG_BOOST_HOLD_MS + 100,
          "declared reasonably close to the hold time, not much later");
}

static void test_boost_ignores_a_single_jolt(void)
{
    printf("test: a single spike does not trigger boost\n");

    prg_boost_t det;
    prg_boost_init(&det);

    for (uint32_t t = 0; t < 2000; t += DT_MS) {
        prg_sample_t s = make(t, 0.0f, true);
        s.accel_g = (t == 1000) ? 5.0f : 1.0f;   /* one spike, then back to normal */
        prg_boost_update(&det, &s);
    }

    CHECK(!det.detected, "single jolt did not trigger boost");
}

static void test_state_machine_full_flight(void)
{
    printf("test: state machine walks a full flight\n");

    prg_flight_t f;
    CHECK(prg_flight_init(&f, "/tmp/perigee_flight_test.bin"), "flight log opened");
    f.state = PRG_STATE_ARMED;

    prg_flight_state_t seen_boost   = PRG_STATE_IDLE;
    prg_flight_state_t seen_descent = PRG_STATE_IDLE;
    prg_flight_state_t seen_landed  = PRG_STATE_IDLE;
    uint32_t boost_t_ms   = 0;
    uint32_t descent_t_ms = 0;
    uint32_t landed_t_ms  = 0;

    uint32_t TOTAL_MS = 20000;   /* long enough to include 10s on the ground */

    for (uint32_t t = 0; t <= TOTAL_MS; t += DT_MS) {
        prg_sample_t s;
        s.t_ms       = t;
        s.baro_valid = true;
        s.imu_valid  = true;

        if (t < 1000) {
            s.accel_g = 1.0f;
            s.alt_m   = 0.0f;
        } else if (t < 3000) {
            s.accel_g = 5.0f;
            s.alt_m   = profile_alt(t);
        } else if (t < 6000) {
            s.accel_g = 0.0f;
            s.alt_m   = profile_alt(t);
        } else {
            s.accel_g = 1.0f;    /* on the ground, at rest */
            s.alt_m   = 0.0f;    /* altitude has stopped changing */
        }

        prg_flight_update(&f, &s);

        if (f.state == PRG_STATE_BOOST && seen_boost == PRG_STATE_IDLE) {
            seen_boost = PRG_STATE_BOOST;
            boost_t_ms = t;
        }
        if (f.state == PRG_STATE_DESCENT && seen_descent == PRG_STATE_IDLE) {
            seen_descent = PRG_STATE_DESCENT;
            descent_t_ms = t;
        }
        if (f.state == PRG_STATE_LANDED && seen_landed == PRG_STATE_IDLE) {
            seen_landed = PRG_STATE_LANDED;
            landed_t_ms = t;
        }
    }

    printf("        BOOST at %u ms, DESCENT at %u ms, LANDED at %u ms\n",
           boost_t_ms, descent_t_ms, landed_t_ms);

    CHECK(seen_boost == PRG_STATE_BOOST, "state reached BOOST");
    CHECK(seen_descent == PRG_STATE_DESCENT, "state reached DESCENT");
    CHECK(seen_landed == PRG_STATE_LANDED, "state reached LANDED");
    CHECK(f.state == PRG_STATE_LANDED, "final state is LANDED");

    prg_log_close(f.log_file);

    FILE *check = fopen("/tmp/perigee_flight_test.bin", "rb");
    CHECK(check != NULL, "flight log file reopened for verification");

    uint32_t record_count   = 0;
    uint32_t boost_seen_t    = 0;
    uint32_t landed_seen_t   = 0;
    bool     boost_found     = false;
    bool     landed_found    = false;

    prg_log_record_t rec;
    while (prg_log_read(check, &rec)) {
        record_count++;

        if (rec.state == (uint8_t)PRG_STATE_BOOST && !boost_found) {
            boost_found = true;
            boost_seen_t = rec.t_ms;
        }
        if (rec.state == (uint8_t)PRG_STATE_LANDED && !landed_found) {
            landed_found = true;
            landed_seen_t = rec.t_ms;
        }
    }
    fclose(check);

    printf("        log contains %u records\n", record_count);

    CHECK(record_count == (TOTAL_MS / DT_MS) + 1,
          "record count matches number of samples fed in");
    CHECK(boost_found && boost_seen_t == boost_t_ms + DT_MS,
          "log shows BOOST one sample after the transition, as designed");
    CHECK(landed_found && landed_seen_t == landed_t_ms + DT_MS,
          "log shows LANDED one sample after the transition, as designed");
}

static void test_burnout_with_noise(void)
{
    printf("test: burnout detection with sensor noise\n");

    prg_burnout_t d;
    prg_burnout_init(&d);

    unsigned seed = 99;

    /* Burnout is only ever fed samples once boost has already occurred -
       start this test mid-burn, not from the pad. */
    for (uint32_t t = 1100; t < 5000; t += DT_MS) {
        seed = seed * 1103515245u + 12345u;
        float noise = (((float)((seed >> 16) & 0x7fff) / 16383.5f) - 1.0f) * 0.1f;

        prg_sample_t s = make(t, 0.0f, true);
        s.accel_g = (t < 3000) ? (5.0f + noise) : (0.0f + noise);

        prg_burnout_update(&d, &s);
    }

    CHECK(d.detected, "burnout detected under noise");
    if (d.detected) {
        long lag = (long)d.detected_t_ms - 3000L;
        printf("        burnout at 3000 ms, declared %u ms, lag %ld ms\n",
               d.detected_t_ms, lag);
        CHECK(lag >= 0 && lag < 500, "declared soon after true burnout");
    }
}

static void test_landing_with_noise(void)
{
    printf("test: landing detection with sensor noise\n");

    prg_landing_t d;
    prg_landing_init(&d);

    unsigned seed = 7;

    for (uint32_t t = 0; t < 15000; t += DT_MS) {
        seed = seed * 1103515245u + 12345u;
        float alt_noise = (((float)((seed >> 16) & 0x7fff) / 16383.5f) - 1.0f) * 0.3f;

        seed = seed * 1103515245u + 12345u;
        float accel_noise = (((float)((seed >> 16) & 0x7fff) / 16383.5f) - 1.0f) * 0.1f;

        prg_sample_t s = make(t, 100.0f + alt_noise, true);
        s.accel_g = 1.0f + accel_noise;

        prg_landing_update(&d, &s);
    }

    CHECK(d.detected, "landing detected under noise");
    if (d.detected) {
        printf("        declared landed at %u ms\n", d.detected_t_ms);
        CHECK(d.detected_t_ms >= PRG_LANDING_HOLD_MS,
              "not declared before the minimum possible hold time");
    }
}

static void test_log_write_and_read_back(void)
{
    printf("test: log write/read round trip\n");

    const char *path = "/tmp/perigee_test_log.bin";

    FILE *out;
    CHECK(prg_log_open(&out, path), "log file opened for writing");

    for (uint32_t t = 0; t < 5; t++) {
        prg_log_record_t rec;
        rec.t_ms       = t * 20u;
        rec.alt_m      = (float)t * 1.5f;
        rec.accel_g    = 1.0f;
        rec.baro_valid = 1u;
        rec.imu_valid  = 1u;
        rec.state      = PRG_STATE_COAST;

        CHECK(prg_log_write(out, &rec), "record written");
    }
    prg_log_close(out);

    FILE *check = fopen(path, "rb");
    CHECK(check != NULL, "log file reopened for reading");

    int count = 0;
    prg_log_record_t rec;
    while (prg_log_read(check, &rec)) {
        CHECK(rec.t_ms == (uint32_t)(count * 20), "timestamp matches expected sequence");
        CHECK(rec.state == PRG_STATE_COAST, "state matches what was written");
        count++;
    }
    fclose(check);

    printf("        read back %d records\n", count);
    CHECK(count == 5, "read back exactly 5 records");
}

static void test_log_next_path_skips_existing(void)
{
    printf("test: log_next_path finds the next unused filename\n");

    /* clean slate - remove anything left over from a previous run */
    unlink("flight_001.bin");
    unlink("flight_002.bin");
    unlink("flight_003.bin");

    char path[64];

    CHECK(prg_log_next_path(path, sizeof(path)), "found a path with nothing existing");
    CHECK(strcmp(path, "flight_001.bin") == 0, "first call returns flight_001.bin");

    /* create flight_001.bin so it's "already taken" */
    FILE *f1 = fopen("flight_001.bin", "wb");
    CHECK(f1 != NULL, "created flight_001.bin to simulate an existing flight");
    fclose(f1);

    CHECK(prg_log_next_path(path, sizeof(path)), "found a path with flight_001 taken");
    CHECK(strcmp(path, "flight_002.bin") == 0, "second call skips to flight_002.bin");

    /* also take flight_002.bin */
    FILE *f2 = fopen("flight_002.bin", "wb");
    CHECK(f2 != NULL, "created flight_002.bin");
    fclose(f2);

    CHECK(prg_log_next_path(path, sizeof(path)), "found a path with 001 and 002 taken");
    CHECK(strcmp(path, "flight_003.bin") == 0, "third call skips to flight_003.bin");

    /* clean up after ourselves */
    unlink("flight_001.bin");
    unlink("flight_002.bin");
}

static void test_flight_init_auto_does_not_overwrite(void)
{
    printf("test: prg_flight_init_auto avoids overwriting a previous flight\n");

    /* clean slate */
    unlink("flight_001.bin");
    unlink("flight_002.bin");

    prg_flight_t f1;
    CHECK(prg_flight_init_auto(&f1), "first flight initialized");

    prg_sample_t s;
    s.t_ms = 0; s.alt_m = 0.0f; s.accel_g = 1.0f;
    s.baro_valid = true; s.imu_valid = true;
    prg_flight_update(&f1, &s);
    prg_log_close(f1.log_file);

    FILE *check1 = fopen("flight_001.bin", "rb");
    CHECK(check1 != NULL, "first flight created flight_001.bin");
    if (check1) fclose(check1);

    prg_flight_t f2;
    CHECK(prg_flight_init_auto(&f2), "second flight initialized");

    prg_flight_update(&f2, &s);
    prg_log_close(f2.log_file);

    FILE *check2 = fopen("flight_002.bin", "rb");
    CHECK(check2 != NULL, "second flight created flight_002.bin, not overwriting flight_001");
    if (check2) fclose(check2);

    /* prove flight_001.bin still has real content, not erased by the second flight */
    FILE *verify1 = fopen("flight_001.bin", "rb");
    prg_log_record_t rec;
    bool got_one = prg_log_read(verify1, &rec);
    fclose(verify1);
    CHECK(got_one, "flight_001.bin still contains its original record, untouched");

    unlink("flight_001.bin");
    unlink("flight_002.bin");
}

static void test_fake_i2c_bus_read_write(void)
{
    printf("test: fake I2C bus read/write mechanics\n");

    prg_i2c_fake_t fake;
    prg_i2c_fake_init(&fake, 0x77);

    prg_i2c_bus_t bus = prg_i2c_fake_as_bus(&fake);

    /* manually seed a register, as if a real chip had this value burned in */
    fake.registers[0x00] = 0x50;

    uint8_t chip_id = 0;
    bool ok = bus.read(&bus, 0x77, 0x00, &chip_id, 1);

    CHECK(ok, "read call reports success");
    CHECK(chip_id == 0x50, "read back the chip ID we seeded");

    /* reading the wrong device address should fail */
    uint8_t junk = 0;
    bool wrong_addr_ok = bus.read(&bus, 0x68, 0x00, &junk, 1);
    CHECK(!wrong_addr_ok, "reading the wrong address correctly fails");

    /* multi-byte read across several registers */
    fake.registers[0x04] = 0x11;
    fake.registers[0x05] = 0x22;
    fake.registers[0x06] = 0x33;

    uint8_t multi[3] = {0, 0, 0};
    ok = bus.read(&bus, 0x77, 0x04, multi, 3);
    CHECK(ok, "multi-byte read succeeds");
    CHECK(multi[0] == 0x11 && multi[1] == 0x22 && multi[2] == 0x33,
          "multi-byte read returns bytes in the right order");

    /* writing through the interface should change what a later read sees */
    uint8_t to_write = 0x99;
    ok = bus.write(&bus, 0x77, 0x10, &to_write, 1);
    CHECK(ok, "write call reports success");
    CHECK(fake.registers[0x10] == 0x99, "write actually changed the underlying register");
}

static void test_bmp388_check_id(void)
{
    printf("test: BMP388 chip ID check\n");

    prg_i2c_fake_t fake;
    prg_i2c_fake_init(&fake, PRG_BMP388_ADDR);
    prg_i2c_bus_t bus = prg_i2c_fake_as_bus(&fake);

    fake.registers[PRG_BMP388_REG_CHIP_ID] = PRG_BMP388_CHIP_ID_VAL;
    CHECK(prg_bmp388_check_id(&bus), "genuine chip ID is accepted");

    fake.registers[PRG_BMP388_REG_CHIP_ID] = 0xFF;
    CHECK(!prg_bmp388_check_id(&bus), "wrong chip ID is correctly rejected");
}

static void test_bmp388_read_raw(void)
{
    printf("test: BMP388 raw pressure/temperature read\n");

    prg_i2c_fake_t fake;
    prg_i2c_fake_init(&fake, PRG_BMP388_ADDR);
    prg_i2c_bus_t bus = prg_i2c_fake_as_bus(&fake);

    /* pressure bytes, low to high: 0x01, 0x02, 0x03 */
    fake.registers[PRG_BMP388_REG_DATA + 0] = 0x01;
    fake.registers[PRG_BMP388_REG_DATA + 1] = 0x02;
    fake.registers[PRG_BMP388_REG_DATA + 2] = 0x03;

    /* temperature bytes, low to high: 0xAA, 0xBB, 0xCC */
    fake.registers[PRG_BMP388_REG_DATA + 3] = 0xAA;
    fake.registers[PRG_BMP388_REG_DATA + 4] = 0xBB;
    fake.registers[PRG_BMP388_REG_DATA + 5] = 0xCC;

    prg_bmp388_raw_t raw;
    bool ok = prg_bmp388_read_raw(&bus, &raw);

    CHECK(ok, "raw read succeeds");

    uint32_t expected_pressure = 0x030201u;
    uint32_t expected_temp     = 0xCCBBAAu;

    printf("        pressure: got 0x%06X, expected 0x%06X\n",
           raw.raw_pressure, expected_pressure);
    printf("        temperature: got 0x%06X, expected 0x%06X\n",
           raw.raw_temperature, expected_temp);

    CHECK(raw.raw_pressure == expected_pressure,
          "pressure bytes combined in the correct order");
    CHECK(raw.raw_temperature == expected_temp,
          "temperature bytes combined in the correct order");
}

static void test_mpu6050_read_and_convert(void)
{
    printf("test: MPU6050 read and convert\n");

    prg_i2c_fake_t fake;
    prg_i2c_fake_init(&fake, PRG_MPU6050_ADDR);
    prg_i2c_bus_t bus = prg_i2c_fake_as_bus(&fake);

    fake.registers[PRG_MPU6050_REG_WHO_AM_I] = PRG_MPU6050_WHO_AM_I_VAL;
    CHECK(prg_mpu6050_check_id(&bus), "genuine chip ID accepted");

    fake.registers[PRG_MPU6050_REG_WHO_AM_I] = 0x00;
    CHECK(!prg_mpu6050_check_id(&bus), "wrong chip ID rejected");
    fake.registers[PRG_MPU6050_REG_WHO_AM_I] = PRG_MPU6050_WHO_AM_I_VAL;

    CHECK(prg_mpu6050_wake(&bus), "wake write succeeds");
    CHECK(fake.registers[PRG_MPU6050_REG_PWR_MGMT_1] == 0x00,
          "wake actually cleared PWR_MGMT_1");

    /* accel_x = +16384 (should read as +1.0 g)   -> 0x40, 0x00 */
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 0]  = 0x40;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 1]  = 0x00;
    /* accel_y = -16384 (should read as -1.0 g)   -> 0xC0, 0x00 */
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 2]  = 0xC0;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 3]  = 0x00;
    /* accel_z = 0 */
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 4]  = 0x00;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 5]  = 0x00;
    /* temp_raw = 0 -> should convert to 36.53 C exactly */
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 6]  = 0x00;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 7]  = 0x00;
    /* gyro_x = 131 (should read as 1.0 deg/s)    -> 0x00, 0x83 */
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 8]  = 0x00;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 9]  = 0x83;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 10] = 0x00;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 11] = 0x00;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 12] = 0x00;
    fake.registers[PRG_MPU6050_REG_ACCEL_DATA + 13] = 0x00;

    prg_mpu6050_raw_t raw;
    CHECK(prg_mpu6050_read_raw(&bus, &raw), "raw read succeeds");

    CHECK(raw.accel_x == 16384,  "accel_x raw matches (positive)");
    CHECK(raw.accel_y == -16384, "accel_y raw matches (negative, sign correct)");
    CHECK(raw.gyro_x  == 131,    "gyro_x raw matches");

    prg_mpu6050_data_t data;
    prg_mpu6050_convert(&raw, &data);

    printf("        accel_x: %.3f g (expect 1.000)\n", data.accel_x_g);
    printf("        accel_y: %.3f g (expect -1.000)\n", data.accel_y_g);
    printf("        temp: %.3f C (expect 36.530)\n", data.temp_c);
    printf("        gyro_x: %.3f deg/s (expect 1.000)\n", data.gyro_x_dps);

    CHECK(data.accel_x_g > 0.99 && data.accel_x_g < 1.01, "accel_x converts to ~1.0 g");
    CHECK(data.accel_y_g < -0.99 && data.accel_y_g > -1.01, "accel_y converts to ~-1.0 g");
    CHECK(data.temp_c > 36.5 && data.temp_c < 36.6, "temp converts to ~36.53 C at raw=0");
    CHECK(data.gyro_x_dps > 0.99 && data.gyro_x_dps < 1.01, "gyro_x converts to ~1.0 deg/s");
}

int main(void)
{
    printf("\nPerigee host tests\n==================\n\n");

    test_detects_on_clean_profile();
    test_no_false_positive_climbing();
    test_single_glitch_rejected();
    test_velocity_detector();
    test_velocity_detector_with_noise();
    test_boost_detects_ignition();
    test_boost_ignores_a_single_jolt();
    test_state_machine_full_flight();
    test_landing_with_noise();
    test_burnout_with_noise();
    test_log_write_and_read_back();
    test_log_next_path_skips_existing();
    test_flight_init_auto_does_not_overwrite();
    test_fake_i2c_bus_read_write();
    test_bmp388_check_id();
    test_bmp388_read_raw();
    test_mpu6050_read_and_convert();
    printf("\n%s (%d failures)\n\n",
           failures ? "FAILED" : "ALL PASSED", failures);

    return failures ? 1 : 0;
}
