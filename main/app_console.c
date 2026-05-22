#include "app_console.h"

#include <errno.h>
#include <stdarg.h>
#include <string.h>

static volatile bool console_enabled;

bool app_console_is_enabled(void)
{
    return console_enabled;
}

void app_console_set_enabled(bool enabled)
{
    console_enabled = enabled;
}

int app_console_printf(const char *format, ...)
{
    if (!console_enabled) {
        return 0;
    }

    va_list args;
    va_start(args, format);
    int ret = vprintf(format, args);
    va_end(args);
    return ret;
}

int app_console_fprintf(FILE *stream, const char *format, ...)
{
    if (!console_enabled) {
        return 0;
    }

    va_list args;
    va_start(args, format);
    int ret = vfprintf(stream, format, args);
    va_end(args);
    return ret;
}

void app_console_perror(const char *message)
{
    if (!console_enabled) {
        return;
    }

    int err = errno;
    fprintf(stderr, "%s: %s\n", message, strerror(err));
}
