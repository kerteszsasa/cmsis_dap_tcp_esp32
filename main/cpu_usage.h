#ifndef CPU_USAGE_H
#define CPU_USAGE_H

#ifdef __cplusplus
extern "C" {
#endif

// Priority fot the task.
#define CPU_USAGE_TASK_PRIO     3

void cpu_usage_task(void* arg);

#ifdef __cplusplus
}
#endif

#endif
