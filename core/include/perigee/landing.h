#ifndef PERIGEE_LANDING_H
#define PERIGEE_LANDING_H

#include "perigee/sample.h"

#define PRG_LANDING_ALT_WINDOW_M     2.0f
#define PRG_LANDING_ACCEL_WINDOW_G   0.5f
#define PRG_LANDING_REST_ACCEL_G     1.0f
#define PRG_LANDING_HOLD_MS          10000u

typedef struct {
    bool     have_ref_alt;
    float    ref_alt_m;
    bool     stable;
    uint32_t stable_since_ms;
    bool     detected;
    uint32_t detected_t_ms;
} prg_landing_t;

void prg_landing_init(prg_landing_t *d);
bool prg_landing_update(prg_landing_t *d, const prg_sample_t *s);

#endif