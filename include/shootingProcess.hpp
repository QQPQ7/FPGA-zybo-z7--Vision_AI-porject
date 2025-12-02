#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <optional>
#include <filesystem>
#include <future>
#include <cstdio>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>          // ✅ ShootingExecutor에서 사용

#include "realtime_utils.hpp"

extern std::atomic<bool> g_reset; 

namespace fs = std::filesystem;

// ==========================================================
// 🚀 유틸리티 (Utility)
// ==========================================================

static bool path_exists(const std::string& p) {
    struct stat st{};
    return ::stat(p.c_str(), &st) == 0;
}

static int write_text(const std::string& p, const std::string& s) {
    int fd = ::open(p.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        std::perror(("open " + p).c_str());
        return -1;
    }
    ssize_t n = ::write(fd, s.c_str(), s.size());
    if (n < 0 || (size_t)n != s.size()) {
        std::perror(("write " + p).c_str());
        ::close(fd);
        return -1;
    }
    ::close(fd);
    return 0;
}

// ==========================================================
// 💡 GpioOut 클래스: GPIO 제어
// ==========================================================

class GpioOut {
public:
    explicit GpioOut(int gpio, bool active_low=false)
        : gpio_(gpio), active_low_(active_low) {}

    bool init() {
        const std::string gp = "/sys/class/gpio/gpio" + std::to_string(gpio_);
        if (!path_exists(gp)) {
            if (write_text("/sys/class/gpio/export", std::to_string(gpio_)) < 0) {
                std::fprintf(stderr, "export %d failed\n", gpio_);
                return false;
            }
        }
        for (int i = 0; i < 50 && !path_exists(gp); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (write_text(gp + "/direction", "out") < 0) return false;
        (void)write_text(gp + "/active_low", active_low_ ? "1" : "0");
        value_path_ = gp + "/value";
        value_fd_ = ::open(value_path_.c_str(), O_WRONLY | O_CLOEXEC);
        if (value_fd_ < 0) {
            std::perror(("open " + value_path_).c_str());
            return false;
        }
        return true;
    }

    void on()  { set(true);  }
    void off() { set(false); }

    void set(bool high) {
        if (value_fd_ < 0) return;
        const char* s = high ? "1" : "0";
        ssize_t n = ::pwrite(value_fd_, s, 1, 0);
        if (n != 1) {
            ::close(value_fd_);
            value_fd_ = ::open(value_path_.c_str(), O_WRONLY | O_CLOEXEC);
            if (value_fd_ >= 0) {
                (void)::pwrite(value_fd_, s, 1, 0);
            } else {
                std::perror(("reopen " + value_path_).c_str());
            }
        }
    }

    ~GpioOut() {
        if (value_fd_ >= 0) ::close(value_fd_);
    }

private:
    int gpio_;
    bool active_low_;
    std::string value_path_;
    int value_fd_{-1};
};

// ==========================================================
// 💡 Ina219Hwmon 클래스: INA219 전력 센서 읽기
// ==========================================================

struct Ina219Sample { int mA{-1}; };

class Ina219Hwmon {
public:
    explicit Ina219Hwmon(std::string device_hint = "") {
        base_ = find_hwmon_base(device_hint);
        ok_ = !base_.empty();
        if (!ok_) {
            std::cerr << "[INA219] hwmon path not found\n";
        }
    }

    bool ok() const { return ok_; }

    std::optional<Ina219Sample> read_once() const {
        if (!ok_) return std::nullopt;
        Ina219Sample s;
        auto cur = read_int_file(base_ + "/curr1_input");
        if (cur) s.mA = *cur;
        if (s.mA == -1) return std::nullopt;
        return s;
    }

private:
    static std::optional<int> read_int_file(const std::string& p){
        std::ifstream ifs(p);
        if (!ifs) return std::nullopt;
        long long v = 0;
        ifs >> v;
        if (!ifs.good() && !ifs.eof()) return std::nullopt;
        return static_cast<int>(v);
    }

    static std::string read_first_line(const fs::path& p){
        std::ifstream ifs(p);
        std::string line;
        std::getline(ifs, line);
        return line;
    }

    static std::string find_hwmon_base(const std::string& hint) {
        if (!hint.empty()) {
            fs::path try1 = fs::path("/sys/bus/i2c/devices") / hint / "hwmon";
            if (fs::exists(try1) && fs::is_directory(try1)) {
                for (auto& d : fs::directory_iterator(try1)) {
                    if (fs::is_directory(d)) {
                        fs::path name = d.path() / "name";
                        if (fs::exists(name)) {
                            auto n = read_first_line(name);
                            if (n.rfind("ina", 0) == 0) { return d.path().string(); }
                        } else { 
                            return d.path().string(); 
                        }
                    }
                }
            }
        }

        fs::path root = "/sys/class/hwmon";
        if (fs::exists(root) && fs::is_directory(root)) {
            for (auto& d : fs::directory_iterator(root)) {
                fs::path name = d.path() / "name";
                if (!fs::exists(name)) continue;
                auto n = read_first_line(name);
                if (n == "ina2xx" || n == "ina219") { 
                    return d.path().string(); 
                }
            }
        }
        return {};
    }

    std::string base_;
    bool ok_{false};
};

// ==========================================================
// 🔧 INA219 사용 여부 토글 매크로
// ==========================================================
// 0 : INA219 전혀 사용하지 않음 (IMU I2C 디버그용, shooting은 타이머로만)
// 1 : INA219 전류 드롭으로 shooting 완료 감지
#ifndef USE_INA219_SHOOTING
#define USE_INA219_SHOOTING 1
#endif

// ==========================================================
// ⚙️ shootingOnce 함수: 발사 동작 로직
// ==========================================================

bool shootingOnce(GpioOut& je1, const Ina219Hwmon& ina) {
#if USE_INA219_SHOOTING

    if (!ina.ok()) {
        std::cerr << "Error: INA219 sensor is not ready.\n";
        return false;
    }

    std::cout << "--- Shooting Start ---\n";
    int  preCurrent       = 0;
    bool shootingComplete = false;

    auto t_start = std::chrono::steady_clock::now();

    // 1. 발사 시작: JE1 ON
    je1.on();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (g_reset.load()) {
        std::cout << "[ESTOP] abort shooting (before loop)\n";
        je1.off();                 // 총구 GPIO 무조건 LOW
        return false;              // 비정상 종료 표시
    }

    // 2. 전류 변화 감지 루프
    while (!shootingComplete) {


        if (g_reset.load()) {
            std::cout << "[ESTOP] abort shooting (before loop)\n";
            je1.off();                 // 총구 GPIO 무조건 LOW
            return false;              // 비정상 종료 표시
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - t_start).count();
        if (elapsed_ms >= 3000) {
            std::cout << "[TIMEOUT] shooting loop exceeded 3 seconds. Abort.\n";
            shootingComplete = true;
            break;  // 루프 종료 (타임아웃)
        }


        auto s = ina.read_once();
        if (!s) {
            std::cerr << "Warning: Could not read INA219 data. Retrying...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 안전 상한선 (예시)
        if (s->mA > 100) {
            std::cout << "Warning: readina > 100mA\n";
            je1.off();
            break;
        }

        // 전류 드롭으로 shooting 완료 감지
        if (s->mA - preCurrent <= -15) {
            shootingComplete = true;
            std::cout << "Shooting detected! Current drop: "
                      << (s->mA - preCurrent) << " mA\n";
        }

        preCurrent = s->mA;
        std::cout << "Current: " << s->mA << " mA\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    // 3. 발사 종료: JE1 OFF
    je1.off();
    std::cout << "--- Shooting Complete ---\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;

#else   // USE_INA219_SHOOTING == 0  ➜ INA219 완전 비활성화 모드

    (void)ina; // 경고 방지

    std::cout << "--- Shooting Start (NO INA219 MODE) ---\n";

    // 1. 발사 시작: JE1 ON
    je1.on();

    // 원래 루프가 대략 2초 정도 도는 느낌이라면,
    // 100ms (초기) + 1900ms ≈ 2.0초 정도로 고정
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::this_thread::sleep_for(std::chrono::milliseconds(1900));

    // 2. (전류 감지는 안 하고) 타이머 기반으로만 종료
    je1.off();

    std::cout << "--- Shooting Complete (NO INA219 MODE) ---\n";
    return true;

#endif
}

// ==========================================================
//   ShootingExecutor 클래스: 1회 발사 스레드 실행 및 종료
// ==========================================================

class ShootingExecutor {
public:
    ShootingExecutor(GpioOut& je1, const Ina219Hwmon& ina)
        : je1_(je1), ina_(ina), running_(false) {}

    ~ShootingExecutor() {
        stop_and_join();
    }

    // 비동기 발사 시작
    bool shoot_async() {
        // 이전 스레드가 남아있으면 join
        stop_and_join();

        // 이미 실행 중이면 재실행 방지
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            std::cerr << "[Executor] Shooting is already running.\n";
            return false;
        }

        worker_thread_ = std::thread(&ShootingExecutor::thread_func, this);
        // set_realtime_priority(worker_thread_, 20, 0); // CPU 0, FIFO 우선순위 20
        return true;
    }

    // 외부에서 "발사 끝날 때까지 기다리기"를 원하면 호출
    void wait() {
        stop_and_join();
    }

private:
    GpioOut& je1_;
    const Ina219Hwmon& ina_;

    std::thread       worker_thread_;
    std::atomic<bool> running_;

    void thread_func() {
        shootingOnce(je1_, ina_);
        running_.store(false);
    }

    void stop_and_join() {
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        running_.store(false);
    }
};
