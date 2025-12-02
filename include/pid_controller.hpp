#pragma once
#include <cmath>
#include <algorithm>

struct PIDConfig {
    float ang_sign = 1.0f;
    float setpoint_deg = 0.0f;
    float kp = 1.5f;
    float ki = 0.1f;
    float kd = 0.5f;
    float d_sign = 1.0f;
    float deadband_deg = 1.0f;
    float max_step_deg = 10.0f;
    float ang_min_deg = -90.0f;
    float ang_max_deg = 45.0f;
};

class PIDController {
public:
    explicit PIDController(const PIDConfig& cfg) : cfg_(cfg), integral_(0.0f) {}
    void reset(double ts, float pitch_raw_deg) {
        last_ts_ = ts;
        last_raw_deg_ = pitch_raw_deg;
        last_meas_deg_ = cfg_.ang_sign * pitch_raw_deg;
        last_rate_dps_ = 0.0f;
        last_target_deg_ = cfg_.setpoint_deg;
        integral_ = 0.0f;
    }
    void set_setpoint(float sp_deg) {
        cfg_.setpoint_deg = sp_deg;
        last_target_deg_ = sp_deg;
    }
    void set_gains(float kp, float ki, float kd) {
        cfg_.kp = kp;
        cfg_.ki = ki;
        cfg_.kd = kd;
    }
    void set_deadband(float db_deg) {
        cfg_.deadband_deg = db_deg;
    }
    float get_setpoint() const { return cfg_.setpoint_deg; } // 추가
    float last_meas_deg() const { return last_meas_deg_; }
    float last_rate_dps() const { return last_rate_dps_; }
    float last_integral() const { return integral_; }
    float update(double ts, float pitch_raw_deg, float gyro_dps) {
        float dt = static_cast<float>(ts - last_ts_);
        if (dt <= 0.0f) dt = 0.01f;
        const float meas_deg = cfg_.ang_sign * pitch_raw_deg;
        const float rate_dps = cfg_.ang_sign * gyro_dps;
        last_rate_dps_ = 0.9f * last_rate_dps_ + 0.1f * rate_dps; // 필터링 강화
        float err = cfg_.setpoint_deg - meas_deg;
        if (std::fabs(err) < cfg_.deadband_deg) err = 0.0f;
        integral_ += err * dt;
        integral_ = clampf(integral_, -50.0f, 50.0f);
        float command_offset = cfg_.kp * err + cfg_.ki * integral_ - (cfg_.kd * cfg_.d_sign) * last_rate_dps_;
        float target = cfg_.setpoint_deg + command_offset;
        const float prev = last_target_deg_;
        const float d = target - prev;
        if (d > cfg_.max_step_deg) target = prev + cfg_.max_step_deg;
        if (d < -cfg_.max_step_deg) target = prev - cfg_.max_step_deg;
        target = clampf(target, cfg_.ang_min_deg, cfg_.ang_max_deg);
        last_ts_ = ts;
        last_raw_deg_ = pitch_raw_deg;
        last_meas_deg_ = meas_deg;
        last_target_deg_ = target;
        return target;
    }
private:
    static float clampf(float x, float lo, float hi) {
        return std::max(lo, std::min(x, hi));
    }
    PIDConfig cfg_;
    double last_ts_ = 0.0;
    float last_raw_deg_ = 0.0f;
    float last_meas_deg_ = 0.0f;
    float last_rate_dps_ = 0.0f;
    float last_target_deg_ = 0.0f;
    float integral_ = 0.0f;
};