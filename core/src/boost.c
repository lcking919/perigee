#include "perigee/boost.h"

void prg_boost_init(prg_boost_t *d)
{
    d->above_threshold = false;
    d->above_since_ms  = 0u;
    d->detected        = false;
    d->detected_t_ms   = 0u;
}

bool prg_boost_update(prg_boost_t *d, const prg_sample_t *s)
{
    if (d->detected) {
        return true;
    }
    if (!s->imu_valid) {
        return false;
    }

    if (s->accel_g >= PRG_BOOST_THRESHOLD_G) {
        if (!d->above_threshold) {
            d->above_threshold = true;
            d->above_since_ms  = s->t_ms;
        }

        uint32_t held_ms = s->t_ms - d->above_since_ms;
        if (held_ms >= PRG_BOOST_HOLD_MS) {
            d->detected      = true;
            d->detected_t_ms = s->t_ms;
        }
    } else {
        d->above_threshold = false;
    }

    return d->detected;
}