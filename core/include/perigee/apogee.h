#ifndef PERIGEE_APOGEE_H
#define PERIGEE_APOGEE_H

#include "perigee/sample.h"

#define PRG_APOGEE_DROP_M     1.0f
#define PRG_APOGEE_CONFIRM_N  3

typedef struct {
    float    max_alt_m;
    uint32_t max_alt_t_ms;
    uint8_t  below_count;
    bool     detected;
    uint32_t detected_t_ms;
} prg_apogee_t;

void prg_apogee_init(prg_apogee_t *d);
bool prg_apogee_update(prg_apogee_t *d, const prg_sample_t *s);

#endif
