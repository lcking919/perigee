#include "perigee/state.h"

void prg_flight_init(prg_flight_t *f)
{
    f->state = PRG_STATE_IDLE;
    prg_boost_init(&f->boost);
    prg_burnout_init(&f->burnout);
    prg_vapogee_init(&f->apogee);
}

bool prg_flight_update(prg_flight_t *f, const prg_sample_t *s)
{
    switch (f->state) {

    case PRG_STATE_IDLE:
        break;

    case PRG_STATE_ARMED:
        if (prg_boost_update(&f->boost, s)) {
            f->state = PRG_STATE_BOOST;
        }
        break;

    case PRG_STATE_BOOST:
        if (prg_burnout_update(&f->burnout, s)) {
            f->state = PRG_STATE_COAST;
        }
        break;

    case PRG_STATE_COAST:
        if (prg_vapogee_update(&f->apogee, s)) {
            f->state = PRG_STATE_DESCENT;
        }
        break;

    case PRG_STATE_DESCENT:
        if (prg_landing_update(&f->landing, s)) {
            f->state = PRG_STATE_LANDED;
        }
        break;

    case PRG_STATE_LANDED:
        break;
    }

    return true;
}