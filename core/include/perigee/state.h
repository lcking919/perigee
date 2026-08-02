#ifndef PERIGEE_STATE_H
#define PERIGEE_STATE_H

#include <stdio.h>
#include "perigee/sample.h"
#include "perigee/vapogee.h"
#include "perigee/boost.h"
#include "perigee/burnout.h"
#include "perigee/landing.h"
#include "perigee/log.h"




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
    prg_burnout_t      burnout;
    prg_vapogee_t      apogee;
    prg_landing_t      landing;
    void               *log_handle;
} prg_flight_t;

bool prg_flight_init(prg_flight_t *f, const char *log_path);
bool prg_flight_update(prg_flight_t *f, const prg_sample_t *s);
bool prg_flight_init_auto(prg_flight_t *f);

#endif