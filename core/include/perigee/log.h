#ifndef PERIGEE_LOG_H
#define PERIGEE_LOG_H

#include <stdio.h>
#include "perigee/sample.h"

typedef struct {
    uint32_t t_ms;
    float    alt_m;
    float    accel_g;
    uint8_t  baro_valid;
    uint8_t  imu_valid;
    uint8_t  state;
} prg_log_record_t;

bool prg_log_open(FILE **f, const char *path);
bool prg_log_write(FILE *f, const prg_log_record_t *rec);
bool prg_log_read(FILE *f, prg_log_record_t *rec);
bool prg_log_next_path(char *buf, size_t buf_len);
void prg_log_close(FILE *f);

#endif