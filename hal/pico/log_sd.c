#include "perigee/log.h"
#include "ff.h"
#include <stdio.h>
#include <stdlib.h>

bool prg_log_next_path(char *buf, size_t buf_len)
{
    for (int n = 1; n <= 999; n++) {
        int written = snprintf(buf, buf_len, "0:flight_%03d.bin", n);
        if (written < 0 || (size_t)written >= buf_len) {
            return false;
        }

        FILINFO info;
        FRESULT fr = f_stat(buf, &info);

        if (fr == FR_NO_FILE) {
            return true;
        }
        if (fr != FR_OK) {
            return false;
        }
    }
    return false;
}

bool prg_log_open(void **handle, const char *path)
{
    FIL *file = malloc(sizeof(FIL));
    if (file == NULL) {
        return false;
    }

    FRESULT fr = f_open(file, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        free(file);
        return false;
    }

    *handle = (void *)file;
    return true;
}

bool prg_log_open_read(void **handle, const char *path)
{
    FIL *file = malloc(sizeof(FIL));
    if (file == NULL) {
        return false;
    }

    FRESULT fr = f_open(file, path, FA_READ);
    if (fr != FR_OK) {
        free(file);
        return false;
    }

    *handle = (void *)file;
    return true;
}

bool prg_log_write(void *handle, const prg_log_record_t *rec)
{
    FIL *file = (FIL *)handle;
    if (file == NULL) {
        return false;
    }

    UINT written;
    FRESULT fr = f_write(file, rec, sizeof(prg_log_record_t), &written);

    if (fr != FR_OK || written != sizeof(prg_log_record_t)) {
        return false;
    }

    return true;
}

bool prg_log_read(void *handle, prg_log_record_t *rec)
{
    FIL *file = (FIL *)handle;
    if (file == NULL) {
        return false;
    }

    UINT got;
    FRESULT fr = f_read(file, rec, sizeof(prg_log_record_t), &got);

    if (fr != FR_OK || got != sizeof(prg_log_record_t)) {
        return false;
    }

    return true;
}

void prg_log_close(void *handle)
{
    FIL *file = (FIL *)handle;
    if (file == NULL) {
        return;
    }

    f_close(file);
    free(file);
}

bool prg_log_write_blob(void *handle, const void *data, size_t len)
{
    FIL *file = (FIL *)handle;
    if (file == NULL) {
        return false;
    }

    UINT written;
    FRESULT fr = f_write(file, data, len, &written);

    if (fr != FR_OK || written != len) {
        return false;
    }

    return true;
}