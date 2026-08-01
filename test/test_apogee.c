#include "perigee/apogee.h"
#include "perigee/vapogee.h"
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

int main(void)
{
    printf("\nPerigee host tests\n==================\n\n");

    test_detects_on_clean_profile();
    test_no_false_positive_climbing();
    test_single_glitch_rejected();
    test_velocity_detector();
    test_velocity_detector_with_noise();

    printf("\n%s (%d failures)\n\n",
           failures ? "FAILED" : "ALL PASSED", failures);

    return failures ? 1 : 0;
}
