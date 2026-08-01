#ifndef PERIGEE_BURNOUT_H
#define PERIGEE_BURNOUT_H

#include "perigee/sample.h"

#define PRG_BURNOUT_THRESHOLD_G   1.5f
#define PRG_BURNOUT_HOLD_MS       100u

typedef struct {
    bool     below_threshold;
    uint32_t below_since_ms;
    bool     detected;
    uint32_t detected_t_ms;
} prg_burnout_t;

void prg_burnout_init(prg_burnout_t *d);
bool prg_burnout_update(prg_burnout_t *d, const prg_sample_t *s);

#endif