#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>
#include <future>
#include <optional>
//#include <condition_variable>

#include "gimbalstable.hpp"
#include "hcsr04_sensor.hpp"
#include "shootingProcess.hpp" // GpioOut, ShootingExecutor, Ina219Hwmon 포함

#include "network_service.hpp"
#include "network_types.hpp"

#include "imu.hpp"
#include "imu_data.h"

#include <random> 

// 편의를 위한 네임스페이스
using namespace std::chrono_literals;

extern std::atomic<bool> g_run;


class WeaponMaster {
private:

    IMUReader& imu_;
    std::atomic<bool> initialize_gimbal_degree{false}; // 김발 각도 초기화용 플래그


     // Weapon_Master가 외부에서 참조하는 객체들
    NetworkService& net;      // UDP 통신 서비스 (콜백 설정 및 메시지 송신)
    BBoxData& sharedBBox;     // Vision 결과(BBox) 공유 데이터 (GimbalStabilizer와 Vision_App_Host가 공유)
    std::mutex& sharedBBox_mtx;
    std::atomic<int>& op_mode; // 운용 모드
    
    // Weapon_Master가 소유하고 관리하는 장비 객체들
    GimbalStabilizer stabilizer; // 짐벌 안정화 및 추적 컨트롤러
    GpioOut je1_shooting;
    Ina219Hwmon current_sensor; // 전류 센서
    ShootingExecutor shooting_executor;   // 총 발사 동작 실행기
    GpioOut je3_laser;           // 레이저 on/off GPIO 제어
    Hcsr04Sensor sonar;          // HCSR04 초음파 센서

    // Weapon_Master가 gimbal stable 스레드 관리
    std::thread stabilizer_thread;


    // // Fire Ready 전용 스레드
    // std::thread fire_thread_;
    // std::atomic<bool> fire_thread_run_{false};

    // // 이벤트 알림용 플래그 + CV
    // std::mutex fire_mtx_;
    // std::condition_variable fire_cv_;
    // bool fire_request_ = false;   // "Fire Ready 이벤트가 하나 있다"는 의미
    
    /*
     * @brief 짐벌 초기화, 레이저 ON 등 하드웨어 초기화 확인
     */
    void init()
    {
        // je1_shooting 초기화 검증
        if (!je1_shooting.init()) {
            std::fprintf(stderr, "JE1 GPIO init failed.\n");
        }

        // Ina219Hwmon 초기화 검증
        if (!current_sensor.ok()) {
            std::cerr << "INA219 initialization failed.\n";
        }

        // je3_laser 초기화 검증
        if (!je3_laser.init()) {
            std::fprintf(stderr, "JE3 GPIO init failed.\n");
        }

        je3_laser.on(); // 레이져 켜기

    

        if (sonar.init() != 0) {
            std::fprintf(stderr, "[WARN] HCSR04 init failed.\n");
        }

        if (!stabilizer.init()){
            std::cerr << "[FATAL] gimbal stabilizer init failed\n";
            g_run.store(false);
        } else {
            // 기본 목표 각도(오프셋) — 필요 시 PC 명령/타깃 기반으로 실시간 갱신
            stabilizer.set_target_angles(-4.0f, -1.0f);
        }
    }

    /*
     * @brief 초음파 센서 측정 명령을 비동기적으로 실행하여 거리를 얻습니다. (Hcsr04Sensor::measure_async 위임)
     */
    double get_distance()
    {
        std::future<double> future_result = sonar.measure_async_avg(5);
        // get()을 호출하여 비동기 작업이 완료될 때까지 기다리고 결과를 반환
        return future_result.get();
    }

    // void notify_fire_ready_event() {
    //     {
    //         std::lock_guard<std::mutex> lk(fire_mtx_);
    //         fire_request_ = true;   // 이벤트 플래그 세우기
    //     }
    //     fire_cv_.notify_one();      // 워커 스레드 깨우기
    // }
    
    // void fire_ready_worker() {
    //     while (fire_thread_run_.load() && g_run.load()) {
    //         std::unique_lock<std::mutex> lk(fire_mtx_);
    //         fire_cv_.wait(lk, [&]{
    //             return fire_request_ || !fire_thread_run_.load() || !g_run.load();
    //         });

    //         if (!fire_thread_run_.load() || !g_run.load()) {
    //             break; // 종료 플래그면 바로 탈출
    //         }

    //         // 이벤트 하나 소비
    //         fire_request_ = false;
    //         lk.unlock();

    //         // 🔥 실제 Fire Ready 처리 (예전엔 김발 스레드에서 돌던 것)
    //         handle_fire_ready();
    //     }
    // }

    void handle_fire_ready()
    {
        // 1. 거리 측정 (get_distance()에서 Future.get()을 통해 동기적 대기 및 측정)
        double distance = get_distance();

        if (!(distance > 0.0)) {  // 0 이하, NaN 모두 포함
            static thread_local std::mt19937 rng(std::random_device{}());
            static thread_local std::uniform_real_distribution<double> dist_rand(100.0, 150.0);

            double fallback = dist_rand(rng);
            std::cerr << "[INFO] distance invalid, fallback random: "
                    << fallback << " cm\n";
            distance = fallback;
        }
        
        // 2. Fire Ready 메시지 송신
        int target_id = 0;
        {
            std::lock_guard<std::mutex> lock(sharedBBox_mtx);
            // sharedBBox.objects[0]이 유효한지 확인하는 로직 추가 필요
            target_id = sharedBBox.objects.empty() ? 0 : sharedBBox.objects[0].id;
        }
        net.send_fire_ready({target_id, true, distance});

        // 3. 발사 결정 (200.0보다 멀면 사거리 밖으로 판단)
        if (op_mode.load(std::memory_order_acquire) == 1) {
            shoot();
            net.send_fire_done({target_id});
            
            initialize_gimbal_degree = true;
        } else {
            std::cerr << "[WARN] Target too far/close (" << distance << " cm). Not firing.\n";
        }
    }

public:
    // 생성자
    /*
     * @brief Weapon_Master 클래스의 생성자입니다. 모든 하드웨어/서비스 객체를 초기화하고 연결합니다.
     * @param shooting_executor 총 발사 실행 객체
     * @param je3_laser 레이저 GPIO 출력 객체
     * @param je4_relay 릴레이 GPIO 출력 객체
     * @param sonar 초음파 센서 객체
     * @param net 네트워크 서비스 객체 (참조로 받음)
     * @param sharedBBox 비전 BBox 공유 데이터 (참조로 받음)
     * @param op_mode 운용 모드 (참조로 받음)
     * @param stabilizer 김발 안정화 및 추적 객체
     */

    WeaponMaster(int je1_gpio, bool je1_active_level, const std::string& ina_hint,
                 int je3_gpio, bool je3_active_level,
                 int trig, int echo,
                 NetworkService& net_, BBoxData& bbox_data, std::mutex& shared_bbox_mutex_ref,
                 std::atomic<int>& operation_mode, IMUReader& imu_reader)
    : net(net_),
      sharedBBox(bbox_data),
      sharedBBox_mtx(shared_bbox_mutex_ref),
      op_mode(operation_mode),
      imu_(imu_reader),
      stabilizer(imu_, sharedBBox, sharedBBox_mtx, op_mode, initialize_gimbal_degree),
    
      je1_shooting(je1_gpio, je1_active_level), // 1. GpioOut 멤버를 인자로 초기화
      current_sensor(ina_hint),             // 2. Ina219Hwmon 멤버를 인자로 초기화
      shooting_executor(je1_shooting, current_sensor), // 3. 위의 두 객체를 '이미 초기화된 상태'로 참조하여 초기화
      je3_laser(je3_gpio, je3_active_level),
      sonar(trig, echo)
    {

    }

    // 소멸자
    ~WeaponMaster() {
        stop();
    }
 
    /*
     * @brief 초기화 및 스레드 동작 시작
     */
    bool run()
    {
        init();

        // 🔔 이제 콜백은 handle_fire_ready가 아니라 "알림 함수"로 바인딩
        // auto callback_func = std::bind(&WeaponMaster::notify_fire_ready_event, this);
        auto callback_func = std::bind(&WeaponMaster::handle_fire_ready, this);
        stabilizer.set_ready_callback(callback_func);

        if (g_run.load()) {
            // // 1) Fire Ready 워커 스레드 먼저 시작
            // fire_thread_run_.store(true);
            // fire_thread_ = std::thread(&WeaponMaster::fire_ready_worker, this);

            // 2) 짐벌 안정화 스레드 시작
            stabilizer_thread = std::thread(&GimbalStabilizer::stabilize_loop, &stabilizer);
            return true;
        }
        return false;
    }


    /*
     * @brief 모든 스레드를 안전하게 종료하고 하드웨어를 해제(deinit)합니다.
     */
    void stop()
    {
        g_run.store(false);

        // // 🔚 Fire Ready 스레드 종료 신호
        // fire_thread_run_.store(false);
        // {
        //     std::lock_guard<std::mutex> lk(fire_mtx_);
        //     fire_request_ = true;   // 깨워주기용
        // }
        // fire_cv_.notify_all();
        // if (fire_thread_.joinable()) fire_thread_.join();

        // 🔚 짐벌 스레드 종료
        stabilizer.request_stop();
        if (stabilizer_thread.joinable()) stabilizer_thread.join();
        stabilizer.deinit();

        je3_laser.off();
    }

      void enter_estop() {
        // 1) 모든 액추에이터 안전 OFF
        gpio_control(1, false); // cannon off (JE1)

        std::cout<<"Enter stop\n";
        // 2) 짐벌 제어 루프에 "정지 모드" 알리기
        initialize_gimbal_degree.store(true);

    }

    // 기능 메서드
    /*
     * @brief GPIO를 on/off 설정을 변경합니다.
     */
    void gpio_control(int gpio_num, bool on) {
        switch (gpio_num)
        {
        case 1:
            if (on) je1_shooting.on();
            else je1_shooting.off();
            break;
        case 3:
            if (on) je3_laser.on();
            else je3_laser.off();
            break;
        default:
            break;
        }
    }

    /*
     * @brief 총 발사 명령을 비동기적으로 실행합니다. (ShootingExecutor::shoot_async 위임)
     */
    void shoot() {
        std::cout<<"[shoot opmode]"<<op_mode<<"\n";
        shootingOnce(je1_shooting, current_sensor);
    }

    /*
     * @brief 김발의 각도를 offset 크기만큼 변경 시킵니다. (GimbalStabilizer::shift_target_angles 위임)
     */
    void shifting_angles(float yaw_deg_offset, float pitch_deg_offset) {
        stabilizer.shift_target_angles(yaw_deg_offset, pitch_deg_offset);
    }

    /*
     * @brief 김발의 현재 Yaw 각도를 가져옵니다. (GimbalStabilizer::get_yaw() 위임)
     * @return 김발의 현재 Yaw 각도 (degree/s)
     */
    float get_yaw() {
        return stabilizer.get_yaw();
    }

    float get_pitch(){
        return stabilizer.get_pitch();
    }
};