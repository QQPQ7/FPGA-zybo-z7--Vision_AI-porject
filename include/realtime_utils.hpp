#pragma once

#include <thread>
#include <pthread.h>
#include <sched.h>
#include <iostream>
#include <cstring>

// ⚡️ 스레드 우선순위 + 코어 고정 유틸
inline void set_realtime_priority(std::thread& th, int priority, int cpu_core) {
    pthread_t handle = th.native_handle();

    // CPU affinity
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    int ret = pthread_setaffinity_np(handle, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "Failed to set CPU affinity: " << strerror(ret) << "\n";
    }

    // 실시간 FIFO 우선순위
    sched_param sch{};
    sch.sched_priority = priority;
    ret = pthread_setschedparam(handle, SCHED_FIFO, &sch);
    if (ret != 0) {
        std::cerr << "Failed to set thread priority: " << strerror(ret) << "\n";
    }
}