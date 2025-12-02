#ifndef GIMBAL_STABLE_HPP
#define GIMBAL_STABLE_HPP

#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <atomic>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <vector>
#include <locale.h>
#include <optional>
#include <functional>
#include "imu.hpp"
#include "imu_data.h"
#include "servo_pwm.hpp"
#include "pid_controller.hpp"
#include "network_types.hpp"
#include "realtime_utils.hpp"

using namespace std::chrono_literals;

struct TrackAngles {
    float yaw_deg;
    float pitch_deg;
};

// ====== 하드웨어/서보 고정 상수 ======
static constexpr int PWM_CHIP_YAW = 0;
static constexpr int PWM_CH_YAW = 0;
static constexpr int PWM_CHIP_PITCH = 1;
static constexpr int PWM_CH_PITCH = 0;
static constexpr unsigned SERVO_PERIOD_NS = 20'000'000U; // 50Hz

static constexpr float YAW_ANG_MIN = -90.0f;
static constexpr float YAW_ANG_MAX = 90.0f;
static constexpr float PITCH_ANG_MIN = -90.0f;
static constexpr float PITCH_ANG_MAX = 90.0f;

static constexpr auto LOOP_DT = 10ms;

// ====== 오프셋/방향 (고정) ======
static constexpr float YAW_OFFSET_DEG = -4.0f;
static constexpr float PITCH_OFFSET_DEG = -1.0f;
static constexpr int YAW_DIR = -1;
static constexpr int PITCH_DIR = -1;

// ====== PID 고정 계수 ======
static constexpr float PID_KP = 0.3f;
static constexpr float PID_KI = 3.0f;
static constexpr float PID_KD = 0.0075f;

// ====== 유틸 ======
static inline float clamp_deg(float x, float lo, float hi) {
    return std::max(lo, std::min(x, hi));
}

static float calculate_imu_bias(IMUReader& imu, int samples = 200, std::chrono::milliseconds dt = 10ms) {
    // 첫 샘플 2초 대기
    {
        IMUData tmp{}; double ts = 0.0;
        auto start = std::chrono::steady_clock::now();
        while (true) {
            if (imu.latest(tmp, ts, nullptr)) break;
            if (std::chrono::steady_clock::now() - start > 2s) {
                std::fprintf(stderr, "[GimbalStabilizer] IMU 첫 샘플 타임아웃(2s) → 바이어스 0 사용\n");
                return 0.0f;
            }
            std::this_thread::sleep_for(5ms);
        }
    }
    std::vector<float> angles;
    angles.reserve(samples);
    double ts = 0.0;
    IMUData d{};
    for (int i = 0; i < samples; ++i) {
        if (imu.latest(d, ts, nullptr)) {
            angles.push_back(d.mAngles[1]); // Pitch 각도(도)
        } else {
            --i;
            std::this_thread::sleep_for(dt);
            continue;
        }
        std::this_thread::sleep_for(dt);
    }
    if (angles.empty()) return 0.0f;
    float sum = 0.0f;
    for (float a : angles) sum += a;
    return -sum / static_cast<float>(angles.size());
}

// ====== 짐벌 안정화 클래스 ======
class GimbalStabilizer {
public:
    GimbalStabilizer(IMUReader& imu, BBoxData& bboxdata, std::mutex& shared_bbox_mutex_ref, std::atomic<int>& operation_mode, std::atomic<bool>& init_flag_ref)
        : imu_(imu),
          yaw_servo_(),
          pitch_servo_(),
          pid_([&]() {
              PIDConfig cfg;
              cfg.ang_sign = 1.0f;
              cfg.setpoint_deg = 0.0f; // 초기 setpoint은 0
              cfg.kp = PID_KP;
              cfg.ki = PID_KI;
              cfg.kd = PID_KD;
              cfg.d_sign = 1.0f;
              cfg.deadband_deg = 1.0f;
              cfg.max_step_deg = 10.0f;
              cfg.ang_min_deg = PITCH_ANG_MIN;
              cfg.ang_max_deg = PITCH_ANG_MAX;
              return PIDController(cfg);
          }()),
          desired_yaw_deg_(YAW_OFFSET_DEG),
          desired_pitch_deg_(0.0f),
          pitch_bias_(0.0f),
          sharedBBox(bboxdata),
          sharedBBox_mtx(shared_bbox_mutex_ref),
          is_fire_ready_called_(false),
          run_(true),
          operation_mode_(operation_mode),

          initialize_gimbal_degree(init_flag_ref),

          vision_cx_(0.0f),
          vision_cy_(0.0f),
          vision_w_(0.0f),
          vision_h_(0.0f),
          prev_pitch_deg_(0.0f),
          prev_ts_(0.0),
          first_sample_(true)
          {          }

    void request_stop() { run_.store(false); imu_.stop(); }

    bool init() {
        if (yaw_servo_.init(PWM_CHIP_YAW, PWM_CH_YAW, SERVO_PERIOD_NS) < 0) {
            std::fprintf(stderr, "[GimbalStabilizer] Yaw 서보 초기화 실패\n");
            return false;
        }
        if (pitch_servo_.init(PWM_CHIP_PITCH, PWM_CH_PITCH, SERVO_PERIOD_NS) < 0) {
            std::fprintf(stderr, "[GimbalStabilizer] Pitch 서보 초기화 실패\n");
            return false;
        }
        yaw_servo_.set_angle_deg(desired_yaw_deg_, YAW_ANG_MIN, YAW_ANG_MAX, YAW_OFFSET_DEG, YAW_DIR);
        pitch_servo_.set_angle_deg(desired_pitch_deg_, PITCH_ANG_MIN, PITCH_ANG_MAX, PITCH_OFFSET_DEG, PITCH_DIR);
        return true;
    }

    

    // Vision thread에서 bbox 갱신
    void update_bbox(float cx, float cy, float w, float h) {
        vision_cx_.store(cx);
        vision_cy_.store(cy);
        vision_w_.store(w);
        vision_h_.store(h);
    }

    void set_target_angles(float yaw_deg, float pitch_deg) {
        desired_yaw_deg_ = clamp_deg(yaw_deg, YAW_ANG_MIN, YAW_ANG_MAX);
        desired_pitch_deg_ = clamp_deg(pitch_deg, PITCH_ANG_MIN, PITCH_ANG_MAX);
        pid_.set_setpoint(desired_pitch_deg_);
    }

    void shift_target_angles(float yaw_deg_offset, float pitch_deg_offset) {
        desired_yaw_deg_ = clamp_deg(desired_yaw_deg_ + yaw_deg_offset, YAW_ANG_MIN, YAW_ANG_MAX);
        desired_pitch_deg_ = clamp_deg(desired_pitch_deg_ + pitch_deg_offset, PITCH_ANG_MIN, PITCH_ANG_MAX);
        pid_.set_setpoint(desired_pitch_deg_);
    }

    void set_ready_callback(std::function<void()> cb) {
        fire_ready_cb_ = std::move(cb);
    }


    void stabilize_loop() {
        IMUData d{}; double ts = 0.0;
        auto next_wakeup = std::chrono::steady_clock::now();
        printf("[Stabilize] 시작 → 목표 Pitch: %.1f°\n", desired_pitch_deg_);
        first_sample_    = true;
        auto last_track = std::chrono::steady_clock::now();
        int last_mode = operation_mode_.load();   // 🆕 직전 모드 저장

        while (run_.load()) {
            next_wakeup += LOOP_DT;
            std::this_thread::sleep_until(next_wakeup);

            // ===== IMU 고장 상태 확인 =====
            bool imu_fault = g_imu_fault.load();
            int  mode      = operation_mode_.load();

            // === ① 수동(0) → 자동(1) 전환: PID 출력에 맞춰 desired 동기화 ===
            if (!imu_fault && mode == 1 && last_mode == 0) {
                // 수동 모드에서 쓰던 마지막 PID 출력(last_pitch_cmd_)를
                // 자동 모드의 desired_pitch_deg_로 맞춰서 명령 불연속 없애기
                desired_pitch_deg_ = clamp_deg(last_pitch_cmd_, PITCH_ANG_MIN, PITCH_ANG_MAX);
                pid_.set_setpoint(desired_pitch_deg_);  // 나중에 수동 복귀 시 기준도 맞춰두기

                std::puts("[GimbalStabilizer] Manual→Auto: desired_pitch_deg_ synced to last PID output");
            }

            // === ② 자동(1) → 수동(0) 전환: IMU pitch에 맞춰 desired 동기화 ===
            if (!imu_fault && mode == 0 && last_mode == 1) {
                // 현재 자세(IMU pitch)를 기준으로 수동 모드의 기준각을 맞춘다.
                if (imu_.latest(d, ts, nullptr)) {
                    float pitch = d.mAngles[1];

                    // 목표각과 PID setpoint를 현재 자세에 맞춤 → 수동 진입 시 튐 방지
                    desired_pitch_deg_ = clamp_deg(pitch, PITCH_ANG_MIN, PITCH_ANG_MAX);
                    pid_.set_setpoint(desired_pitch_deg_);

                    // 다음 수동 루프에서 reset을 실행하도록 플래그 ON
                    first_sample_ = true;
                } else {
                    // IMU 읽기 실패 시 최소한 reset만 하도록
                    first_sample_ = true;
                }
                std::puts("[GimbalStabilizer] Auto→Manual: desired_pitch_deg_ synced to IMU pitch");
            }

            last_mode = mode;

            if (!imu_fault && mode == 0) {
                // ---------- 정상 모드: IMU + PID 안정화 ----------
                if (!imu_.latest(d, ts, nullptr)) {
                    continue;
                }

                float pitch = d.mAngles[1];

                // 첫 샘플은 PID 초기화만
                if (first_sample_) {
                    pid_.reset(ts, pitch);
                    prev_pitch_deg_ = pitch;
                    prev_ts_        = ts;
                    first_sample_   = false;
                } else {
                    double dt = ts - prev_ts_;
                    if (dt <= 0.0 || dt > 0.1) {
                        dt = LOOP_DT.count() / 1000.0;
                    }

                    // pitch 변화량으로 각속도 추정
                    const float pitch_rate_dps =
                        static_cast<float>((pitch - prev_pitch_deg_) / dt);

                    // PID 출력 (목표 pitch로 보정된 값)
                    const float pitch_target =
                        pid_.update(ts, pitch, pitch_rate_dps);

                    yaw_servo_.set_angle_deg(
                        desired_yaw_deg_, YAW_ANG_MIN, YAW_ANG_MAX,
                        YAW_OFFSET_DEG, YAW_DIR
                    );
                    pitch_servo_.set_angle_deg(
                        pitch_target, PITCH_ANG_MIN, PITCH_ANG_MAX,
                        PITCH_OFFSET_DEG, PITCH_DIR
                    );

                    last_pitch_cmd_ = pitch_target;
                    // yaw 정보 업데이트 (PC 송신용)
                    gyro_pitch_dps = d.mAngles[1];
                    gyro_yaw_dps = d.mAngles[2];

                    // prev 상태 갱신
                    prev_pitch_deg_ = pitch;
                    prev_ts_        = ts;
                }
            } else {
                // ---------- FAILSAFE 모드: IMU 없이 단순 서보 ----------
                // IMU는 더 이상 쓰지 않고,
                // Vision에서 갱신되는 desired_yaw_deg_/desired_pitch_deg_만 서보에 반영

                // ----------------imu on / stabilize off----------------
                if (!imu_.latest(d, ts, nullptr)) {
                    continue;
                }
                gyro_pitch_dps = d.mAngles[1];
                gyro_yaw_dps = d.mAngles[2];
                // ----------------imu on / stabilize off----------------


                yaw_servo_.set_angle_deg(
                    desired_yaw_deg_, YAW_ANG_MIN, YAW_ANG_MAX,
                    YAW_OFFSET_DEG, YAW_DIR
                );
                pitch_servo_.set_angle_deg(
                    desired_pitch_deg_, PITCH_ANG_MIN, PITCH_ANG_MAX,
                    PITCH_OFFSET_DEG, PITCH_DIR
                );

                // ----------------imu on / stabilize off----------------
                prev_pitch_deg_ = gyro_pitch_dps;
                prev_ts_        = ts;
                // ----------------imu on / stabilize off----------------

            }

            // ---------- Vision 기반 타겟 추적 (IMU 유무와 상관 없이 항상) ----------
            if (!initialize_gimbal_degree.load()) {
                auto now = std::chrono::steady_clock::now();
                if (now - last_track > 50ms) {
                    last_track = now;

                    float cx = 0.0f, cy = 0.0f, w = 0.0f, h = 0.0f;
                    bool has_obj = false;

                    {
                        std::lock_guard<std::mutex> lock(sharedBBox_mtx);

                        if (!sharedBBox.objects.empty()) {
                            // ✅ priority가 가장 낮은(=1등) BBoxObject 찾기
                            const BBoxObject* best = nullptr;
                            for (const auto& obj : sharedBBox.objects) {
                                if (obj.id == 0) continue; // 방어코드: id 없는 건 스킵
                                if (!best || obj.priority < best->priority) {
                                    best = &obj;
                                }
                            }

                            if (best) {
                                cx = best->bbox[0];
                                cy = best->bbox[1];
                                w  = best->bbox[2];
                                h  = best->bbox[3];
                                has_obj = true;

                                // 디버깅용 로그
                                std::cout << "[VisionTrack] chosen id=" << best->id
                                        << " pri=" << best->priority
                                        << " bbox=(" << cx << "," << cy
                                        << "," << w  << "," << h  << ")\n";
                            }
                        }
                    } // 🔓 mutex unlock

                    if (has_obj) {
                        auto opt = track_step_from_bbox(cx, cy, w, h);

                        if (operation_mode_.load() == 1 && opt.has_value()) {
                            const auto& t = opt.value();
                            std::printf("[VisionTrack] cx=%.1f cy=%.1f -> target yaw=%.1f pitch=%.1f\n",
                                        cx, cy, t.yaw_deg, t.pitch_deg);

                            set_target_angles(t.yaw_deg, t.pitch_deg);
                        }
                    }
                }
            } else {
                // 짐벌 초기 위치 복귀
                move_to_home();
            }

        }
    }

    std::optional<TrackAngles> track_step_from_bbox(float cx, float cy, float w, float h)
    {
        // Vision 값 유효성 검사
        if (cx <= 0 || cy <= 0 || w <= 0 || h <= 0)
            return std::nullopt;

        constexpr int IMG_W = 640;
        constexpr int IMG_H = 480;
        constexpr float DEADZONE_YAW = 40.0f;
        constexpr float DEADZONE_PITCH = 20.0f;


        const float offset_x = cx - (IMG_W * 0.5f);
        const float offset_y = cy - (IMG_H * 0.5f);

        const bool is_centered_yaw = (std::fabs(offset_x) <= DEADZONE_YAW);
        const bool is_centered_pitch = (std::fabs(offset_y) <= DEADZONE_PITCH);
        const bool is_centered = is_centered_yaw && is_centered_pitch;

        // 🔥 데드존에서 1초 이상 머무를 때만 Fire Ready
        using clock = std::chrono::steady_clock;
        constexpr auto FIRE_HOLD_TIME = 1s;   // 1초

        auto now = clock::now();

        if (is_centered) {
            if (!in_deadzone_) {
                // 데드존에 새로 진입
                in_deadzone_ = true;
                deadzone_enter_time_ = now;
                // 이 시점에는 아직 발사 X
            } else {
                // 이미 데드존 안에 있었음 → 머문 시간 체크
                if (!is_fire_ready_called_ &&
                    now - deadzone_enter_time_ >= FIRE_HOLD_TIME) {
                    if (fire_ready_cb_) {
                        fire_ready_cb_();
                    }
                    is_fire_ready_called_ = true;
                    std::cout << "[TRACKING] Fire Ready (held in deadzone >= 1s).\n";
                }
            }
        } else {
            // 데드존에서 벗어나면 상태 초기화
            in_deadzone_ = false;
            is_fire_ready_called_ = false;
            deadzone_enter_time_ = clock::time_point{};
        }

        // 🔧 자동 모드에서만 보정 값을 계산해 반환
        if (operation_mode_.load() != 1)
            return std::nullopt;

        // ===== Vision 보정 계산 (자동 모드 전용) =====
        constexpr float MAX_STEP = 1.0f;
        constexpr float Kp_yaw = 0.005f;
        constexpr float Kp_pitch = 0.005f;

        float target_yaw = desired_yaw_deg_;
        float target_pitch = desired_pitch_deg_;

        if (!is_centered_yaw) {
            float scale = 1.0f + (std::min(std::fabs(offset_x), 200.0f) / 200.0f);
            float delta_yaw = Kp_yaw * offset_x * scale;
            delta_yaw = std::clamp(delta_yaw, -MAX_STEP, MAX_STEP);
            target_yaw += delta_yaw;
        }

        if (!is_centered_pitch) {
            float delta_pitch = Kp_pitch * offset_y;
            delta_pitch = std::clamp(delta_pitch, -MAX_STEP, MAX_STEP);
            target_pitch += delta_pitch;
        }

        target_yaw = clamp_deg(target_yaw, YAW_ANG_MIN, YAW_ANG_MAX);
        target_pitch = clamp_deg(target_pitch, PITCH_ANG_MIN, PITCH_ANG_MAX);

        // 🔄 여기서 set_targetAngles() 호출 삭제됨!
        return TrackAngles{target_yaw, target_pitch};
    }

    void move_to_home(){
        uint8_t init_finish_flag = 0;

        // 한 번에 0.5도씩만 움직이기 (이전엔 2.0f)
        constexpr float STEP_YAW   = 1.0f;
        constexpr float STEP_PITCH = 1.0f;

        if (desired_yaw_deg_ <= -6.0f) {
            shift_target_angles(STEP_YAW, 0.0f); 
        } else if (desired_yaw_deg_ >= -2.0f) {
            shift_target_angles(-STEP_YAW, 0.0f);
        } else {
            init_finish_flag++;
        }

        if (desired_pitch_deg_ <= -3.0f) {
            shift_target_angles(0.0f, STEP_PITCH); 
        } else if (desired_pitch_deg_ >= 1.0f) {
            shift_target_angles(0.0f, -STEP_PITCH);
        } else {
            init_finish_flag++;
        }


        if (init_finish_flag == 2) {
            set_target_angles(-4.0f, -1.0f);
            initialize_gimbal_degree.store(false); 
            std::cout << "[GimbalStabilizer] Yaw 초기화 완료 (-4.0f, -1.0f). Init 플래그 OFF.\n";
        }
        std::this_thread::sleep_for(100ms); // 한 스텝마다 100ms 쉬기 그대로 유지
    }


    float get_yaw()
    {
        return gyro_yaw_dps;
    }

    float get_pitch()
    {
        return gyro_pitch_dps;
    }

    void deinit() {
        run_.store(false);
        yaw_servo_.deinit();
        pitch_servo_.deinit();
        std::puts("[GimbalStabilizer] 서보와 IMU 해제 완료.");
    }

private:
    IMUReader& imu_;
    float gyro_yaw_dps; // PC에 송신할 status 데이터 갱신용
    float gyro_pitch_dps; // PC에 송신할 status 데이터 갱신용

    ServoPWM yaw_servo_;
    ServoPWM pitch_servo_;
    PIDController pid_;
    float desired_yaw_deg_;
    float desired_pitch_deg_;
    float pitch_bias_;
    BBoxData& sharedBBox;
    std::mutex& sharedBBox_mtx;
    bool is_fire_ready_called_; // flag
    std::atomic<bool> run_;
    std::atomic<int>& operation_mode_;

    // 콜백 함수를 저장할 멤버 변수. (인자 없음, void 반환)
    std::function<void()> fire_ready_cb_;

    std::atomic<bool>& initialize_gimbal_degree; // 0.5초 대기 후 플래그를 ON/OFF할 상태 변수

    // Vision 공유 데이터
    std::atomic<float> vision_cx_;
    std::atomic<float> vision_cy_;
    std::atomic<float> vision_w_;
    std::atomic<float> vision_h_;

    float last_pitch_cmd_ = 0.0f;
    float  prev_pitch_deg_;
    double prev_ts_;
    bool   first_sample_;

    bool in_deadzone_ = false;
    std::chrono::steady_clock::time_point deadzone_enter_time_;

};

#endif // GIMBAL_STABLE_HPP
