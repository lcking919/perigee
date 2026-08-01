#include "perigee/apogee.h"
#include <stdio.h>

int main(void)
{
    float track[] = { 10, 20, 30, 38, 44, 47, 48, 47, 45, 42, 38 };
    int   n       = sizeof(track) / sizeof(track[0]);

    prg_apogee_t det;
    prg_apogee_init(&det);

    for (int i = 0; i < n; i++) {
        prg_sample_t s;
        s.t_ms       = (uint32_t)(i * 100);
        s.alt_m      = track[i];
        s.accel_g    = 0.0f;
        s.baro_valid = true;
        s.imu_valid  = true;

        bool hit = prg_apogee_update(&det, &s);

        printf("t=%4u  alt=%5.1f  peak=%5.1f  count=%u  %s\n",
               s.t_ms, (double)s.alt_m, (double)det.max_alt_m,
               det.below_count, hit ? "<-- APOGEE" : "");
    }

    return 0;
}
