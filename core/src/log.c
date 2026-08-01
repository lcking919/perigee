#include "perigee/log.h"
#include <stdio.h>

bool prg_log_open(FILE **f, const char *path)
{
    *f = fopen(path, "wb");
    return (*f != NULL);
}

bool prg_log_write(FILE *f, const prg_log_record_t *rec)
{
    size_t written = fwrite(rec, sizeof(prg_log_record_t), 1, f);
    return (written == 1);
}

bool prg_log_read(FILE *f, prg_log_record_t *rec)
{
    size_t got = fread(rec, sizeof(prg_log_record_t), 1, f);
    return (got == 1);
}

bool prg_log_next_path(char *buf, size_t buf_len)
{
    for (int n = 1; n <= 999; n++) {
        int written = snprintf(buf, buf_len, "flight_%03d.bin", n);
        if (written < 0 || (size_t)written >= buf_len) {
            return false;
        }

        FILE *test = fopen(buf, "rb");
        if (test == NULL) {
            return true;
        }
        fclose(test);
    }
    return false;
}

void prg_log_close(FILE *f)
{
    fclose(f);
}