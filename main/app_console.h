#ifndef APP_CONSOLE_H
#define APP_CONSOLE_H

#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

bool app_console_is_enabled(void);
void app_console_set_enabled(bool enabled);
int app_console_printf(const char *format, ...);
int app_console_fprintf(FILE *stream, const char *format, ...);
void app_console_perror(const char *message);

#ifdef __cplusplus
}
#endif

#endif
