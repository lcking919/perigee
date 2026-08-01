#include "perigee/vapogee.h"

void prg_vapogee_init(prg_vapogee_t *d)
{
    d->have_prev     = false;
    d->prev_alt_m    = 0.0f;
    d->prev_t_ms     = 0u;
    d->velocity_mps  = 0.0f;
    d->neg_count     = 0u;
    d->detected      = false;
    d->detected_t_ms = 0u;
}

bool prg_vapogee_update(prg_vapogee_t *d, const prg_sample_t *s)
{
    if (d->detected) {
        return true;
    }
    if (!s->baro_valid) {
        return false;
    }

    if (!d->have_prev) {
        d->prev_alt_m = s->alt_m;
        d->prev_t_ms  = s->t_ms;
        d->have_prev  = true;
        return false;
    }

    float dt_s = (float)(s->t_ms - d->prev_t_ms) / 1000.0f;
    if (dt_s <= 0.0f) {
        return false;
    }

    d->velocity_mps = (s->alt_m - d->prev_alt_m) / dt_s;

    d->prev_alt_m = s->alt_m;
    d->prev_t_ms  = s->t_ms;

    if (d->velocity_mps < 0.0f) {
        d->neg_count++;
        if (d->neg_count >= PRG_V_CONFIRM_N) {
            d->detected      = true;
            d->detected_t_ms = s->t_ms;
        }
    } else {
        d->neg_count = 0u;
    }

    return d->detected;
}