#include "perigee/log.h"
#include "pico_hal.h"
#include <stdlib.h>

bool prg_log_open(void **handle, const char *path)
{
    int fd = pico_open(path, LFS_O_WRONLY | LFS_O_CREAT);
    if (fd < 0) {
        return false;
    }

    int *stored_fd = malloc(sizeof(int));
    if (stored_fd == NULL) {
        pico_close(fd);
        return false;
    }
    *stored_fd = fd;
    *handle = (void *)stored_fd;
    return true;
}

bool prg_log_write(void *handle, const prg_log_record_t *rec)
{
    int fd = *(int *)handle;
    lfs_size_t written = pico_write(fd, rec, sizeof(prg_log_record_t));
    return (written == sizeof(prg_log_record_t));
}

bool prg_log_read(void *handle, prg_log_record_t *rec)
{
    int fd = *(int *)handle;
    lfs_size_t got = pico_read(fd, rec, sizeof(prg_log_record_t));
    return (got == sizeof(prg_log_record_t));
}

void prg_log_close(void *handle)
{
    int fd = *(int *)handle;
    pico_close(fd);
    free(handle);
}

bool prg_log_open_read(void **handle, const char *path)
{
    int fd = pico_open(path, LFS_O_RDONLY);
    if (fd < 0) {
        return false;
    }

    int *stored_fd = malloc(sizeof(int));
    if (stored_fd == NULL) {
        pico_close(fd);
        return false;
    }
    *stored_fd = fd;
    *handle = (void *)stored_fd;
    return true;
}