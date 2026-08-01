#include "perigee/state.h"

bool prg_flight_init(prg_flight_t *f, const char *log_path)
{
    f->state = PRG_STATE_IDLE;
    prg_boost_init(&f->boost);
    prg_burnout_init(&f->burnout);
    prg_vapogee_init(&f->apogee);
    prg_landing_init(&f->landing);

    return prg_log_open(&f->log_file, log_path);
}

bool prg_flight_init_auto(prg_flight_t *f)
{
    char path[64];

    if (!prg_log_next_path(path, sizeof(path))) {
        return false;
    }

    return prg_flight_init(f, path);
}

bool prg_flight_update(prg_flight_t *f, const prg_sample_t *s)
{
    prg_log_record_t rec;
    rec.t_ms       = s->t_ms;
    rec.alt_m      = s->alt_m;
    rec.accel_g    = s->accel_g;
    rec.baro_valid = s->baro_valid ? 1u : 0u;
    rec.imu_valid  = s->imu_valid  ? 1u : 0u;
    rec.state      = f->state;
    prg_log_write(f->log_file, &rec);


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