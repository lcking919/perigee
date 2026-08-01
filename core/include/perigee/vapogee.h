#ifndef PERIGEE_VAPOGEE_H
#define PERIGEE_VAPOGEE_H

#include "perigee/sample.h"

#define PRG_V_CONFIRM_N  3

typedef struct {
    bool     have_prev;
    float    prev_alt_m;
    uint32_t prev_t_ms;
    float    velocity_mps;
    uint8_t  neg_count;
    bool     detected;
    uint32_t detected_t_ms;
} prg_vapogee_t;

void prg_vapogee_init(prg_vapogee_t *d);

bool prg_vapogee_update(prg_vapogee_t *d, const prg_sample_t *s);

#endif