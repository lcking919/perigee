#ifndef PERIGEE_VAPOGEE_H
#define PERIGEE_VAPOGEE_H

#include "perigee/sample.h"

#define PRG_V_CONFIRM_N  3
#define PRG_V_WINDOW     10


typedef struct {
    float    velocity_mps;
    uint8_t  neg_count;
    bool     detected;
    uint32_t detected_t_ms;

    float    alt_buf[PRG_V_WINDOW];
    uint32_t t_buf[PRG_V_WINDOW];
    uint8_t  buf_head;
    uint8_t  buf_filled;
} prg_vapogee_t;

void prg_vapogee_init(prg_vapogee_t *d);

bool prg_vapogee_update(prg_vapogee_t *d, const prg_sample_t *s);

#endif