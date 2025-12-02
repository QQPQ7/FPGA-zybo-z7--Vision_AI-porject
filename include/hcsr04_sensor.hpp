#pragma once

#include <iostream>
#include <cstdint>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <thread>
#include <future>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>

class Hcsr04Sensor {
public:
    // 생성자: Trig / Echo GPIO 번호
    Hcsr04Sensor(int trig_pin, int echo_pin)
        : trig_pin_(trig_pin), echo_pin_(echo_pin) {}

    // GPIO export 및 방향 설정
    //  0: 성공, <0: 실패
    inline int init() {
        if (gpio_export(trig_pin_)  < 0) return -1;
        if (gpio_export(echo_pin_)  < 0) return -2;
        // sysfs 노드 생성 기다림
        usleep(20000);

        if (gpio_dir(trig_pin_, "out") < 0) return -3;
        if (gpio_dir(echo_pin_, "in")  < 0) return -4;

        // 기본값 Low
        if (gpio_write(trig_pin_, 0)   < 0) return -5;

        // 안전하게 edge 기본값 none
        (void)gpio_edge(echo_pin_, "none");
        return 0;
    }

    // GPIO unexport (선택)
    inline int deinit() {
        int r1 = gpio_unexport(trig_pin_);
        int r2 = gpio_unexport(echo_pin_);
        return (r1 < 0 || r2 < 0) ? -1 : 0;
    }

    // 단일 측정(cm)
    // 음수 에러코드:
    //  -1 (echo open 실패), -2 (RISING 타임아웃/오류),
    //  -3 (RISING 값 오류), -4 (FALLING 타임아웃/오류), -5 (FALLING 값 오류)
    inline double measure_once(int tmo_rise_ms = 50, int tmo_fall_ms = 50) {
        // 준비
        (void)gpio_write(trig_pin_, 0);
        usleep(2000);

        // 에지 이벤트 설정
        if (gpio_edge(echo_pin_, "both") < 0) {
            // 일부 커널은 "both" 미지원일 수 있음 → rising으로 시도
            (void)gpio_edge(echo_pin_, "rising");
        }

        int fd = gpio_open_value_rd(echo_pin_);
        if (fd < 0) {
            std::fprintf(stderr, "[hcsr04] echo open failed (gpio=%d)\n", echo_pin_);
            return -1;
        }

        // 기존 낙전 이벤트 제거
        (void)gpio_read_value_fd(fd);

        // 트리거 10us
        (void)gpio_write(trig_pin_, 1);
        usleep(10);
        (void)gpio_write(trig_pin_, 0);

        // RISING 대기
        struct pollfd pfd{fd, POLLPRI, 0};
        int pr = ::poll(&pfd, 1, tmo_rise_ms);
        if (pr <= 0) { ::close(fd); return -2; } // timeout or error
        int v = gpio_read_value_fd(fd);
        if (v != 1) {
            // 원하는 RISING이 아닐 수 있음 → 한 번 더 poll
            pr = ::poll(&pfd, 1, tmo_rise_ms);
            if (pr <= 0) { ::close(fd); return -2; }
            v = gpio_read_value_fd(fd);
            if (v != 1) { ::close(fd); return -3; }
        }
        uint64_t t_rise = now_monotonic_ns();

        // FALLING 대기
        pr = ::poll(&pfd, 1, tmo_fall_ms);
        if (pr <= 0) { ::close(fd); return -4; }
        v = gpio_read_value_fd(fd);
        if (v != 0) {
            // 원하는 FALLING이 아닐 수 있음 → 한 번 더 poll
            pr = ::poll(&pfd, 1, tmo_fall_ms);
            if (pr <= 0) { ::close(fd); return -4; }
            v = gpio_read_value_fd(fd);
            if (v != 0) { ::close(fd); return -5; }
        }
        uint64_t t_fall = now_monotonic_ns();
        ::close(fd);

        // 시간 → 거리(cm)
        const uint64_t dur_ns = (t_fall - t_rise);
        const double   dur_us = dur_ns / 1000.0;
        // 왕복/음속 보정 상수: 약 58 us/cm
        const double   dist_cm = dur_us / 58.0;
        return dist_cm;
    }

    /**
     * @brief 초음파를 5회 측정하여 평균값을 비동기적으로 계산합니다.
     * @param num_samples 샘플 수 (기본값 5)
     * @return std::future<double> 5회 평균 거리(cm)를 담을 future 객체.
     */
    inline std::future<double> measure_async_avg(int num_samples = 5) {
        // std::async를 사용하여 새로운 스레드에서 평균 계산 로직을 실행합니다.
        return std::async(std::launch::async, [this, num_samples]() -> double {
            double sum_distance = 0.0;
            int valid_samples = 0;
            const int sleep_ms = 20; // 측정 간격 (20ms)

            for (int i = 0; i < num_samples; ++i) {
                double result = this->measure_once();

                if (result > 0) {
                    // 측정값이 유효할 때 (에러 코드 < 0 아닐 때)만 합산
                    sum_distance += result;
                    valid_samples++;
                }
                
                // 다음 측정을 위해 잠시 대기
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            }
            
            if (valid_samples == 0) {
                std::cerr << "[ERROR] All Hcsr04 measurements failed." << std::endl;
                return -10.0; // 유효한 샘플이 없으면 특정 에러 값 반환
            }

            // 5회 측정값의 평균 계산
            return sum_distance / valid_samples;
        });
    }

private:
    int trig_pin_;
    int echo_pin_;

    // ---------- sysfs helpers ----------
    static inline int write_text(const std::string& p, const std::string& v) {
        int fd = ::open(p.c_str(), O_WRONLY);
        if (fd < 0) { perror(("open " + p).c_str()); return -1; }
        if (::write(fd, v.c_str(), v.size()) < 0) {
            perror(("write " + p).c_str()); ::close(fd); return -1;
        }
        ::close(fd);
        return 0;
    }

    static inline int read_text(const std::string& p, char* buf, size_t n) {
        int fd = ::open(p.c_str(), O_RDONLY);
        if (fd < 0) { perror(("open " + p).c_str()); return -1; }
        int r = ::read(fd, buf, n);
        if (r < 0) { perror(("read " + p).c_str()); ::close(fd); return -1; }
        ::close(fd);
        return r;
    }

    static inline bool exists(const std::string& p) {
        struct stat st{};
        return ::stat(p.c_str(), &st) == 0;
    }

    static inline int gpio_export(int n) {
        std::string gp = "/sys/class/gpio/gpio" + std::to_string(n);
        if (exists(gp)) return 0;
        return write_text("/sys/class/gpio/export", std::to_string(n));
    }

    static inline int gpio_unexport(int n) {
        std::string gp = "/sys/class/gpio/gpio" + std::to_string(n);
        if (!exists(gp)) return 0;
        return write_text("/sys/class/gpio/unexport", std::to_string(n));
    }

    static inline int gpio_dir(int n, const char* dir) {
        return write_text("/sys/class/gpio/gpio" + std::to_string(n) + "/direction", dir);
    }

    static inline int gpio_edge(int n, const char* edge) {
        return write_text("/sys/class/gpio/gpio" + std::to_string(n) + "/edge", edge);
    }

    static inline int gpio_write(int n, int v) {
        return write_text("/sys/class/gpio/gpio" + std::to_string(n) + "/value", v ? "1" : "0");
    }

    static inline int gpio_open_value_rd(int n) {
        std::string p = "/sys/class/gpio/gpio" + std::to_string(n) + "/value";
        int fd = ::open(p.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) perror(("open " + p).c_str());
        return fd;
    }

    static inline int gpio_read_value_fd(int fd) {
        ::lseek(fd, 0, SEEK_SET);
        char c = '0';
        int r = ::read(fd, &c, 1);
        if (r <= 0) return -1;
        return (c == '0') ? 0 : 1;
    }

    static inline uint64_t now_monotonic_ns() {
#if defined(CLOCK_MONOTONIC_RAW)
        const clockid_t cid = CLOCK_MONOTONIC_RAW;
#else
        const clockid_t cid = CLOCK_MONOTONIC;
#endif
        struct timespec ts; ::clock_gettime(cid, &ts);
        return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    }
};
