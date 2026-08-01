#include "perigee/log.h"

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

void prg_log_close(FILE *f)
{
    fclose(f);
}