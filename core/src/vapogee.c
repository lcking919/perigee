#include "perigee/vapogee.h"

void prg_vapogee_init(prg_vapogee_t *d)
{
    d->velocity_mps  = 0.0f;
    d->neg_count     = 0u;
    d->detected      = false;
    d->detected_t_ms = 0u;

    for (int i=0; i < PRG_V_WINDOW; i++) {
        d->alt_buf[i] = 0.0f;
        d->t_buf[i] = 0u;
    }
    d->buf_head      = 0u;
    d->buf_filled    = 0u;
}

static uint8_t ring_index(const prg_vapogee_t *d, uint8_t steps_back)
{
    return (uint8_t)((d->buf_head + PRG_V_WINDOW - 1 - steps_back) % PRG_V_WINDOW);
}

bool prg_vapogee_update(prg_vapogee_t *d, const prg_sample_t *s)
{
    if (d->detected) {
        return true;
    }
    if (!s->baro_valid) {
        return false;
    }

    d->alt_buf[d->buf_head] = s->alt_m;
    d->t_buf[d->buf_head]   = s->t_ms;
    d->buf_head = (uint8_t)((d->buf_head + 1) % PRG_V_WINDOW);
    if (d->buf_filled < PRG_V_WINDOW) {
        d->buf_filled++;
    }

    if (d->buf_filled < PRG_V_WINDOW) {
        return false;   /* not enough history yet to average */
    }

    const uint8_t half = PRG_V_WINDOW / 2;
    float old_alt_sum = 0.0f, new_alt_sum = 0.0f;
    float old_t_sum   = 0.0f, new_t_sum   = 0.0f;

    for (uint8_t i = 0; i < half; i++) {
        uint8_t newer = ring_index(d, i);
        uint8_t older = ring_index(d, (uint8_t)(i + half));

        new_alt_sum += d->alt_buf[newer];
        old_alt_sum += d->alt_buf[older];
        new_t_sum   += (float)d->t_buf[newer];
        old_t_sum   += (float)d->t_buf[older];
    }

    float new_alt_avg = new_alt_sum / (float)half;
    float old_alt_avg = old_alt_sum / (float)half;
    float new_t_avg   = new_t_sum   / (float)half;
    float old_t_avg   = old_t_sum   / (float)half;

    float dt_s = (new_t_avg - old_t_avg) / 1000.0f;
    if (dt_s <= 0.0f) {
        return false;
    }

    d->velocity_mps = (new_alt_avg - old_alt_avg) / dt_s;

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