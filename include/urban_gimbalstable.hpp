// gimbalstable.hpp  (수정된 부분만 강조)
#ifndef GIMBAL_STABLE_HPP
#define GIMBAL_STABLE_HPP

#include <thread>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <cmath>
#include <fstream>  
#include <algorithm>
#include "imu.hpp"
#include "imu_data.h"
#include "servo_pwm.hpp"
#include "pid_controller.hpp"

using namespace std::chrono_literals;

static constexpr auto LOOP_DT = 10ms;
static constexpr float ANG_MIN = -90.0f;
static constexpr float ANG_MAX = 90.0f;

static constexpr int PWM_CHIP_YAW = 0,   PWM_CH_YAW = 0;
static constexpr int PWM_CHIP_PITCH = 1, PWM_CH_PITCH = 0;
static constexpr unsigned SERVO_PERIOD_NS = 20'000'000U;

static constexpr float YAW_OFFSET_DEG = -4.0f;
static constexpr float PITCH_OFFSET_DEG = -1.0f;
static constexpr int YAW_DIR = -1;
static constexpr int PITCH_DIR = -1;

static constexpr float PID_KP = 0.30f;
static constexpr float PID_KI = 3.00f;
static constexpr float PID_KD = 0.0075f;

static inline float clamp_deg(float v, float lo = ANG_MIN, float hi = ANG_MAX) {
    return std::max(lo, std::min(hi, v));
}

class GimbalStabilizer {
public:
    GimbalStabilizer()
        : imu_("HEAD"),
          pid_([&]() {
              PIDConfig cfg{};
              cfg.ang_sign = 1.0f;
              cfg.setpoint_deg = 0.0f;
              cfg.kp = PID_KP; cfg.ki = PID_KI; cfg.kd = PID_KD;
              cfg.d_sign = 1.0f;
              cfg.deadband_deg = 1.0f;
              cfg.max_step_deg = 10.0f;
              cfg.ang_min_deg = ANG_MIN;
              cfg.ang_max_deg = ANG_MAX;
              return PIDController(cfg);
          }()),
          target_yaw_(YAW_OFFSET_DEG),
          target_pitch_(0.0f),
          prev_pitch_(0.0f),
          prev_ts_(0.0),
          first_(true),
          run_(true) {}

    bool init() {
        imu_.setDevice("/dev/i2c-0");
        if (!imu_.init(true)) {
            fprintf(stderr, "[ERROR] IMU 초기화 실패\n");
            return false;
        }
        if (yaw_servo_.init(PWM_CHIP_YAW, PWM_CH_YAW, SERVO_PERIOD_NS) < 0 ||
            pitch_servo_.init(PWM_CHIP_PITCH, PWM_CH_PITCH, SERVO_PERIOD_NS) < 0) {
            fprintf(stderr, "[ERROR] 서보 PWM 초기화 실패\n");
            return false;
        }
        yaw_servo_.set_angle_deg(target_yaw_, ANG_MIN, ANG_MAX, YAW_OFFSET_DEG, YAW_DIR);
        pitch_servo_.set_angle_deg(target_pitch_, ANG_MIN, ANG_MAX, PITCH_OFFSET_DEG, PITCH_DIR);
        printf("[GimbalStabilizer] 초기화 완료 (100Hz 제어)\n");
        return true;
    }

    // 여기만 수정! 이름을 다시 set_target_angles()로 통일
    void set_target_angles(float yaw_deg, float pitch_deg) {
        target_yaw_   = clamp_deg(yaw_deg);
        target_pitch_ = clamp_deg(pitch_deg);
        pid_.set_setpoint(target_pitch_);
    }

        void stabilize_loop() {
        using clock = std::chrono::steady_clock;

        // 🔹 CSV 로그 파일 오픈
        std::ofstream csv("pitch_log.csv");
        if (!csv.is_open()) {
            std::fprintf(stderr, "[ERROR] pitch_log.csv 열기 실패\n");
        } else {
            csv << "time_sec,target_pitch_deg,current_pitch_deg,error_deg\n";
            csv.flush();
        }

        auto t0 = clock::now();
        auto next_wakeup = clock::now();

        IMUData data{};
        double ts = 0.0;

        printf("[Stabilize] 시작 → 초기 목표 Pitch: %.1f°\n", target_pitch_);

        while (run_.load()) {
            next_wakeup += LOOP_DT;
            std::this_thread::sleep_until(next_wakeup);

            if (!imu_.latest(data, ts, nullptr)) continue;

            // 🔹 현재 pitch (IMU 필터 출력)
            float pitch = data.mAngles[1];

            if (first_) {
                pid_.reset(ts, pitch);
                prev_pitch_ = pitch;
                prev_ts_ = ts;
                first_ = false;
                continue;
            }

            double dt = ts - prev_ts_;
            if (dt <= 0.0 || dt > 0.1) {
                dt = LOOP_DT.count() / 1000.0; // fallback (초 단위)
            }

            float pitch_rate_dps = (pitch - prev_pitch_) / static_cast<float>(dt);

            // 🔹 PID 출력: 구동에 쓸 목표 서보 각도
            float output = pid_.update(ts, pitch, pitch_rate_dps);

            // 서보 구동
            yaw_servo_.set_angle_deg(target_yaw_, ANG_MIN, ANG_MAX, YAW_OFFSET_DEG, YAW_DIR);
            pitch_servo_.set_angle_deg(output, ANG_MIN, ANG_MAX, PITCH_OFFSET_DEG, PITCH_DIR);

            // 🔹 오차 계산 (목표 - 현재)
            float err = target_pitch_ - pitch;

            // 🔹 시간(sec 단위)
            double t_sec = std::chrono::duration<double>(clock::now() - t0).count();

            // 🔹 콘솔 출력: 목표 / 현재 / 오차
            printf("T:%.2f° | P:%.2f° | err:%.2f° | rate:%.1f dps | out:%.2f° | I:%.3f\r",
                   target_pitch_, pitch, err, pitch_rate_dps, output, pid_.last_integral());
            fflush(stdout);

            // 🔹 CSV 저장
            if (csv.is_open()) {
                csv << t_sec << ","
                    << target_pitch_ << ","
                    << pitch << ","
                    << err << "\n";
                csv.flush();   // 안전하게 매번 flush (필요하면 나중에 간격 조절 가능)
            }

            prev_pitch_ = pitch;
            prev_ts_ = ts;
        }

        if (csv.is_open()) {
            csv.flush();
            csv.close();
        }

        printf("\n[Stabilize] 루프 종료\n");
    }


    void request_stop() { run_.store(false); imu_.stop(); }
    void deinit() {
        run_.store(false);
        imu_.stop();
        yaw_servo_.deinit();
        pitch_servo_.deinit();
        printf("\n[GimbalStabilizer] 정리 완료.\n");
    }

    IMUReader imu_;          // main에서 직접 접근해야 하므로 public

private:
    ServoPWM yaw_servo_;
    ServoPWM pitch_servo_;
    PIDController pid_;

    float target_yaw_;
    float target_pitch_;

    float prev_pitch_;
    double prev_ts_;
    bool first_;

    std::atomic<bool> run_;
};

#endif // GIMBAL_STABLE_HPP