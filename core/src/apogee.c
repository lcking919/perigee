#include "perigee/apogee.h"

void prg_apogee_init(prg_apogee_t *d)
{
    d->max_alt_m     = -1.0e9f;
    d->max_alt_t_ms  = 0u;
    d->below_count   = 0u;
    d->detected      = false;
    d->detected_t_ms = 0u;
}

bool prg_apogee_update(prg_apogee_t *d, const prg_sample_t *s)
{
    if (d->detected) {
        return true;
    }
    if (!s->baro_valid) {
        return false;
    }

    if (s->alt_m > d->max_alt_m) {
        d->max_alt_m    = s->alt_m;
        d->max_alt_t_ms = s->t_ms;
        d->below_count  = 0u;
        return false;
    }

    if ((d->max_alt_m - s->alt_m) >= PRG_APOGEE_DROP_M) {
        d->below_count++;
        if (d->below_count >= PRG_APOGEE_CONFIRM_N) {
            d->detected      = true;
            d->detected_t_ms = s->t_ms;
        }
    } else {
        d->below_count = 0u;
    }

    return d->detected;
}
