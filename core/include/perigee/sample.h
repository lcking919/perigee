#ifndef PERIGEE_SAMPLE_H
#define PERIGEE_SAMPLE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t t_ms;
    float    alt_m;
    float    accel_g;
    bool     baro_valid;
    bool     imu_valid;
} prg_sample_t;

#endif
