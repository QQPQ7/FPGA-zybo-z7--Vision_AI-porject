#pragma once

#include "servo_pwm.hpp"
#include <cstdio>
#include <algorithm>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <stdexcept>

#include "network_service.hpp"
#include "network_types.hpp"

#include <atomic>
#include <csignal>

// vision_main
#include "vision_data.h"
#include "vision_modules.hpp"
#include "vision_app_host.hpp"

#include <gst/gst.h>
#include <opencv2/dnn.hpp>

using namespace std::chrono_literals;

// Servo configuration constants
struct ServoConfig {
    int chip;
    int channel;
    float offset_deg;
    int direction;
};

static constexpr ServoConfig YAW_CONFIG = {0, 0, -4.0f, -1};   // pwmchip0, pwm0, -4° offset, reversed
static constexpr ServoConfig PITCH_CONFIG = {1, 0, -1.0f, 1};   // pwmchip1, pwm0, -1° offset, forward
static constexpr unsigned SERVO_PERIOD_NS = 20'000'000U;        // 50Hz
static constexpr float STEP_DEG = 1.0f;                         // Angle increment per keypress
static constexpr float ANG_MIN = -90.0f;                       // Min angle
static constexpr float ANG_MAX = 90.0f;                        // Max angle

// Terminal raw mode for non-blocking input
class TermRaw {
    struct termios old_{};
    bool ok_{false};

public:
    TermRaw() {
        if (tcgetattr(STDIN_FILENO, &old_) != 0) {
            return;
        }
        termios raw = old_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 1; // 0.1s timeout
        ok_ = (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0);
    }

    ~TermRaw() {
        if (ok_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &old_);
        }
    }

    bool is_ok() const { return ok_; }
};

// Key input enumeration
enum class Key { None, Up, Down, Left, Right, Quit };

// Read a single keypress (arrow keys or 'q')
Key read_key_once() {
    unsigned char buf[3] = {0};
    ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return Key::None;
    if (n == 1 && (buf[0] == 'q' || buf[0] == 'Q')) return Key::Quit;
    if (n >= 3 && buf[0] == 27 && buf[1] == 91) {
        switch (buf[2]) {
            case 'A': return Key::Up;
            case 'B': return Key::Down;
            case 'C': return Key::Right;
            case 'D': return Key::Left;
            default: return Key::None;
        }
    }
    return Key::None;
}

// Servo controller class
class ServoController {
    ServoPWM yaw_;
    ServoPWM pitch_;
    float yaw_cmd_deg_ = 0.0f;
    float pitch_cmd_deg_ = 0.0f;

public:
    ServoController() = default;

    void init() {
        if (yaw_.init(YAW_CONFIG.chip, YAW_CONFIG.channel, SERVO_PERIOD_NS) < 0) {
            throw std::runtime_error("Failed to initialize yaw servo");
        }
        if (pitch_.init(PITCH_CONFIG.chip, PITCH_CONFIG.channel, SERVO_PERIOD_NS) < 0) {
            throw std::runtime_error("Failed to initialize pitch servo");
        }
        // Set initial angles (0° logical, applying offsets)
        update_yaw(0.0f);
        update_pitch(0.0f);
    }

    void update_yaw(float deg) {
        yaw_cmd_deg_ = std::clamp(deg, ANG_MIN, ANG_MAX);
        if (yaw_.set_angle_deg(yaw_cmd_deg_, ANG_MIN, ANG_MAX, YAW_CONFIG.offset_deg, YAW_CONFIG.direction) < 0) {
            throw std::runtime_error("Failed to set yaw angle");
        }
    }

    void update_pitch(float deg) {
        pitch_cmd_deg_ = std::clamp(deg, ANG_MIN, ANG_MAX);
        if (pitch_.set_angle_deg(pitch_cmd_deg_, ANG_MIN, ANG_MAX, PITCH_CONFIG.offset_deg, PITCH_CONFIG.direction) < 0) {
            throw std::runtime_error("Failed to set pitch angle");
        }
    }

    // Getter methods for current angles
    float get_yaw() const { return yaw_cmd_deg_; }
    float get_pitch() const { return pitch_cmd_deg_; }

    void print_status() const {
        std::printf("Cmd Yaw=%6.1f°, Pitch=%6.1f°\n", yaw_cmd_deg_, pitch_cmd_deg_);
        std::fflush(stdout);
    }

    void deinit() {
        yaw_.deinit();
        pitch_.deinit();
    }
};

static std::atomic<bool> g_run{true};

static void on_sigint(int){
    g_run.store(false); 
}

int main() {
    std::signal(SIGINT, on_sigint);

    const std::string DEV        = "/dev/video0";
    const std::string PC_IP      = "192.168.137.1";   // JPEG RTP 보내는 대상
    const int         VIDEO_PORT = 5600;

    const std::string META_IP    = "192.168.137.1";   // 메타데이터 수신 PC
    const int         META_PORT  = 5000;

    const std::string GIMBAL_IP  = "192.168.2.11";   // 짐벌(또는 프록시)
    const int         GIMBAL_PORT= 5002;

    const std::string MODEL      = "/usr/local/share/models/320_224_re_train2_full_integer_quant_edgetpu.tflite";

    NetworkService net(5000);


    if (!net.start()){
        std::cerr << "network start failed\n";
        return 1;
    }

    // std::atomic<bool> vision_app_running{true};

    // // Vision_App_Host 객체를 스레드에서 실행
    // std::thread vision_app_thread([&]{
    //     try {
    //         Vision_App_Host vision_app(DEV, PC_IP, VIDEO_PORT, META_IP, META_PORT, GIMBAL_IP, GIMBAL_PORT, MODEL, net); // 마지막 인자로 net도 전달
    //         vision_app.run(); // 이 안에서 stream/process 스레드가 돌아감
    //     } catch (const std::exception& e) {
    //         std::cerr << "fatal: " << e.what() << "\n";
    //         vision_app_running.store(false);
    //     }
    // });

    try {
        // Initialize terminal raw mode
        TermRaw raw;
        if (!raw.is_ok()) {
            std::puts("[ERR] Failed to set terminal to raw mode");
            return 1;
        }

        // Initialize servos
        ServoController servo;
        servo.init();
        std::puts("[INFO] Servos initialized at 0°. Use arrows to control, 'q' to quit.");
        servo.print_status();

        // Main loop
        while (true) {
            Key key = read_key_once();
            float yaw_delta = 0.0f;
            float pitch_delta = 0.0f;

            switch (key) {
                case Key::Up:
                    pitch_delta = STEP_DEG;
                    break;
                case Key::Down:
                    pitch_delta = -STEP_DEG;
                    break;
                case Key::Left:
                    yaw_delta = -STEP_DEG;
                    break;
                case Key::Right:
                    yaw_delta = STEP_DEG;
                    break;
                case Key::Quit:
                    servo.deinit();
                    std::puts("[INFO] Servos deinitialized. Exiting.");
                    return 0;
                case Key::None:
                    std::this_thread::sleep_for(10ms);
                    continue;
                default:
                    continue;
            }

            // Update servo angles using current values
            servo.update_yaw(servo.get_yaw() + yaw_delta);
            servo.update_pitch(servo.get_pitch() + pitch_delta);
            servo.print_status();
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[ERR] %s\n", e.what());
        return 1;
    }
    std::cout << "Press Ctrl+C to quit\n";
    while (g_run.load()) std::this_thread::sleep_for(std::chrono::seconds(1));
    // if (vision_app_thread.joinable()) vision_app_thread.join();
    net.stop();
    return 0;
}