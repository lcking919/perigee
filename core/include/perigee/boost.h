#ifndef PERIGEE_BOOST_H
#define PERIGEE_BOOST_H

#include "perigee/sample.h"

#define PRG_BOOST_THRESHOLD_G   3.0f
#define PRG_BOOST_HOLD_MS       100u

typedef struct {
    bool     above_threshold;
    uint32_t above_since_ms;
    bool     detected;
    uint32_t detected_t_ms;
} prg_boost_t;

void prg_boost_init(prg_boost_t *d);
bool prg_boost_update(prg_boost_t *d, const prg_sample_t *s);

#endif