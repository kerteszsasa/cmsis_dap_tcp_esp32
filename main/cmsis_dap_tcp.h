#ifndef CMSIS_DAP_TCP_H
#define CMSIS_DAP_TCP_H

#ifdef __cplusplus
extern "C" {
#endif

//#define DEBUG_PRINTING

#ifdef DEBUG_PRINTING
#define LOG_DEBUG(...) \
{ \
    fprintf(stderr, "cmsis_dap_tcp: "); \
    fprintf(stderr, ##__VA_ARGS__); \
    fprintf(stderr, "\n"); \
}

#else
#define LOG_DEBUG(...) { }
#endif

#define LOG_ERROR(...) \
{ \
    fprintf(stderr, "cmsis_dap_tcp ERROR: "); \
    fprintf(stderr, ##__VA_ARGS__); \
    fprintf(stderr, "\n"); \
}

// Task that runs the TCP server and processes requests and responses.
void cmsis_dap_tcp_task(void* arg);

#ifdef __cplusplus
}
#endif

#endif  // CMSIS_DAP_TCP_H
