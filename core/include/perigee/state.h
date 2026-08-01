#ifndef PERIGEE_STATE_H
#define PERIGEE_STATE_H

#include "perigee/sample.h"
#include "perigee/vapogee.h"
#include "perigee/boost.h"

typedef enum {
    PRG_STATE_IDLE = 0,
    PRG_STATE_ARMED,
    PRG_STATE_BOOST,
    PRG_STATE_COAST,
    PRG_STATE_DESCENT,
    PRG_STATE_LANDED
} prg_flight_state_t;

typedef struct {
    prg_flight_state_t state;
    prg_boost_t        boost;
    prg_vapogee_t      apogee;
} prg_flight_t;

void prg_flight_init(prg_flight_t *f);
bool prg_flight_update(prg_flight_t *f, const prg_sample_t *s);

#endif