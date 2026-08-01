#include "perigee/apogee.h"
#include "perigee/vapogee.h"
#include "perigee/boost.h"
#include "perigee/state.h"
#include "perigee/log.h"
#include <stdio.h>
#include <math.h>


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
    printf("\n%s (%d failures)\n\n",
           failures ? "FAILED" : "ALL PASSED", failures);

    return failures ? 1 : 0;
}
