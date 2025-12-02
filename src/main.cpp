#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

// network_test
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

// weapon_master
#include "weapon_master.hpp"

#include "shootingProcess.hpp"

#include "imu.hpp"
#include "imu_data.h"

std::atomic<bool> g_reset{false};   
std::atomic<bool> g_imu_fault{false};
// ====== 보드별로 맞춰야 하는 부분 ======
static constexpr int JE1_GPIO = 512; // 총알 발사 캐논
static constexpr int JE3_GPIO = 514; // 아두이노 레이저 모듈
static constexpr int trig = 516; // JE5
static constexpr int echo = 518; // JE7
static constexpr bool ACTIVE_LOW = false;
static constexpr bool ACTIVE_HIGH = true;
// =====================================


// ====== bbox 데이터 구조체 ======
struct Cxywh {
    float cx, cy, w, h; // 중심 기반, 픽셀 단위
};

std::atomic<bool> g_run{true};

static void on_sigint(int){
    g_run.store(false); 
}


// mode flag (필요에 따라 atomic 말고 mutex + conditional variable 고려할 것)
std::atomic<int> operation_mode{0}; // 0(default): 수동, 1: 자동


int main(int argc, char** argv) {

    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);

// =====================================================================
//   Vision App Host + NetworkService net + sharedBBox
// =====================================================================

    const std::string DEV        = "/dev/video0";
    const std::string PC_IP      = "192.168.50.50";   // JPEG RTP 보내는 대상
    const int         VIDEO_PORT = 5600;

    const std::string META_IP    = PC_IP;   // 메타데이터 수신 PC
    const int         META_PORT  = 5000;

    const std::string MODEL      = "/usr/local/share/models/Best_3d_super_litemodel_pretrained_full_integer_quant_edgetpu.tflite"; //train5_best_full_integer_quant_edgetpu.tflite"; 
    //Best_3d_super_litemodel_pretrained_full_integer_quant_edgetpu.tflite

    NetworkService net(5000);

    std::atomic<bool> vision_app_running{true};

    Vision_App_Host* pVision_app = nullptr;
    BBoxData sharedBBox;
    // 공유 뮤텍스 생성
    std::mutex sharedBBox_mtx;

    // 1. Vision_App_Host 객체를 메인 스레드에 생성
    Vision_App_Host vision_app(DEV, PC_IP, VIDEO_PORT, META_IP, META_PORT, MODEL, net, sharedBBox, sharedBBox_mtx);

    // 2. vision_app.run() 메서드만 별도의 스레드로 분리
    std::thread vision_app_thread([&vision_app, &pVision_app, &vision_app_running]{
        try {
            pVision_app = &vision_app;
            vision_app.run(); // 이 안에서 stream/process 스레드가 돌아감
        } catch (const std::exception& e) {
            std::cerr << "fatal: " << e.what() << "\n";
            vision_app_running.store(false);
        }
    });

// =====================================================================
//   IMUReader 
// =====================================================================

        // 1. IMUReader 생성
    IMUReader imu_head("HEAD");
    imu_head.setDevice("/dev/i2c-0");  // 너가 기존에 쓰던 경로
    if (!imu_head.init(true)) {
        std::fprintf(stderr, "[GimbalStabilizer] IMU 초기화 실패\n");
        g_imu_fault.store(true);
        // return false;
    }
        // 2. IMU 읽기 스레드 (여기에서 우선순위 최대로 올림)
    std::thread imu_thread(&IMUReader::run_loop, &imu_head, 50.0);
    // set_realtime_priority(imu_thread, 40, 0);  // 예: prio 40, CPU0

// =====================================================================
//   Weapon Master
// =====================================================================

    WeaponMaster weapon_master(JE1_GPIO, ACTIVE_LOW, "",
                                JE3_GPIO, ACTIVE_LOW,
                                trig, echo,
                                net, sharedBBox, sharedBBox_mtx,
                                operation_mode, imu_head);

    weapon_master.run();

// =====================================================================
//   Network Callbacks (NetworkService net 사용)
// =====================================================================

    // 수신(PC->Zynq) 콜백들
    net.OnSoftReset = [&weapon_master](const SoftReset& v){
        std::cout << "[RX] EMERGENCY STOP\n";

        // Vision / 통신 / IMU는 계속 돌아가야 하니까
        // 여기서 프로그램 전체를 끄진 말고, 액추에이터만 제어
        std::thread([&weapon_master](){
            g_reset.store(true);

            // 1) 즉시 안전 상태 + 초기각 이동
            weapon_master.enter_estop();

            // 2) 5초간 정지 유지
            std::this_thread::sleep_for(std::chrono::seconds(5));

            operation_mode.store(0);

            // 4) E-Stop 해제
            g_reset.store(false);
        }).detach(); // 네트워크 스레드 블로킹 안 되게 detach
    };

    net.OnGimbalControl = [&weapon_master](const GimbalControl& v){ // 참조 캡처
        if (g_reset.load()) {
            std::cout << "[RX] gimbal_control ignored (ESTOP)\n";
            return;
        }
        std::cout << "[RX] gimbal_control yaw=" << v.yaw << " pitch=" << v.pitch << "\n";
        weapon_master.shifting_angles(v.yaw, v.pitch);
    };
    net.OnOperationMode = [](const OperationMode& v){
        if (g_reset.load()) {
            std::cout << "[RX] operationMode ignored (ESTOP)\n";
            return;
        }
        std::cout << "[RX] operation_mode mode=" << v.mode << "\n";
        operation_mode = v.mode;
    };
    net.OnFireCommand = [&weapon_master](const FireCommand& v){ // 참조 캡처
        if (g_reset.load()) {
            std::cout << "[RX] fire_command ignored (ESTOP)\n";
            return;
        }
        std::cout << "[RX] fire_command trigger=" << (v.trigger?"true":"false") << "\n";
        switch (v.trigger)
        {
        case true:
            // weapon_master.gpio_control(4, true); // relay
            weapon_master.shoot();
            break;
        default:
            break;
        }
    };
    net.OnLaserControl = [&weapon_master](const LaserControl& v){ // 참조 캡처
        if (g_reset.load()) {
            std::cout << "[RX] OnLaserControl ignored (ESTOP)\n";
            return;
        }
        std::cout << "[RX] laser_control state=" << v.state << "\n";
        // pmod port JE 3번 핀에서 gpio output으로 on/off 할 것
        weapon_master.gpio_control(3, (bool)v.state); // laser
    };
 
    net.ConnectionChanged = [](bool on){
        std::cout << "[NET] " << (on ? "Active" : "Lost") << "\n";
    };

    if (!net.start()){
        std::cerr << "network start failed\n";
        return 1;
    }


    // --- 100ms status 주기 송신 ---
    std::thread th_status([&]{
        using namespace std::chrono;
        while (g_run.load()){
            StatusData s;
            s.yaw = weapon_master.get_yaw();
            s.pitch = weapon_master.get_pitch();
            net.send_status(s);
            std::this_thread::sleep_for(200ms);
        }
    });

    // 종료 정리
    std::cout << "Press Ctrl+C to quit\n";
    while (g_run.load()) std::this_thread::sleep_for(std::chrono::seconds(1));

    // if (g_stabilizer){
    //     g_stabilizer->request_stop();
    // }
    // if (stabilize_thread.joinable()) stabilize_thread.join();
    // if (g_stabilizer){
    //     g_stabilizer->deinit();
    //     g_stabilizer = nullptr;
    // }

    g_run.store(false);
    vision_app.request_stop();

// IMU 정리
    imu_head.stop();
    if (imu_thread.joinable()) imu_thread.join();

    if (th_status.joinable()) th_status.join();

    // if (vision_app_thread.joinable()) vision_app_thread.join();

    net.stop();
    
    return 0;
}