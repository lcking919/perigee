#include "perigee/burnout.h"

void prg_burnout_init(prg_burnout_t *d)
{
    d->below_threshold = false;
    d->below_since_ms  = 0u;
    d->detected        = false;
    d->detected_t_ms   = 0u;
}

bool prg_burnout_update(prg_burnout_t *d, const prg_sample_t *s)
{
    if (d->detected) {
        return true;
    }
    if (!s->imu_valid) {
        return false;
    }

    if (s->accel_g <= PRG_BURNOUT_THRESHOLD_G) {
        if (!d->below_threshold) {
            d->below_threshold = true;
            d->below_since_ms  = s->t_ms;
        }

        uint32_t held_ms = s->t_ms - d->below_since_ms;
        if (held_ms >= PRG_BURNOUT_HOLD_MS) {
            d->detected      = true;
            d->detected_t_ms = s->t_ms;
        }
    } else {
        d->below_threshold = false;
    }

    return d->detected;
}