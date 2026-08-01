#include "perigee/landing.h"

void prg_landing_init(prg_landing_t *d)
{
    d->have_ref_alt    = false;
    d->ref_alt_m       = 0.0f;
    d->stable          = false;
    d->stable_since_ms = 0u;
    d->detected        = false;
    d->detected_t_ms   = 0u;
}

bool prg_landing_update(prg_landing_t *d, const prg_sample_t *s)
{
    if (d->detected) {
        return true;
    }
    if (!s->baro_valid || !s->imu_valid) {
        return false;
    }

    float accel_dev = s->accel_g - PRG_LANDING_REST_ACCEL_G;
    if (accel_dev < 0.0f) {
        accel_dev = -accel_dev;
    }

    bool alt_ok   = false;
    bool accel_ok = (accel_dev <= PRG_LANDING_ACCEL_WINDOW_G);

    if (d->stable) {
        float alt_dev = s->alt_m - d->ref_alt_m;
        if (alt_dev < 0.0f) {
            alt_dev = -alt_dev;
        }
        alt_ok = (alt_dev <= PRG_LANDING_ALT_WINDOW_M);
    }

    if (d->stable && alt_ok && accel_ok) {
        uint32_t held_ms = s->t_ms - d->stable_since_ms;
        if (held_ms >= PRG_LANDING_HOLD_MS) {
            d->detected      = true;
            d->detected_t_ms = s->t_ms;
        }
    } else {
        d->stable          = true;
        d->ref_alt_m       = s->alt_m;
        d->stable_since_ms = s->t_ms;
    }

    return d->detected;
}