#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <nlohmann/json.hpp>

#include "udp_data_link.hpp"
#include "network_types.hpp"

// ISO8601 UTC
inline std::string iso8601_now(){
    using namespace std::chrono;
    auto now = time_point_cast<std::chrono::seconds>(system_clock::now());
    std::time_t t = system_clock::to_time_t(now);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%FT%TZ", std::gmtime(&t));
    return buf;
}

class NetworkService {
public:
    // 다운링크(PC->Zynq) 이벤트 콜백
    std::function<void(const GimbalControl&)>  OnGimbalControl;
    std::function<void(const FireMode&)>       OnFireMode;
    std::function<void(const OperationMode&)>  OnOperationMode;
    std::function<void(const FireCommand&)>    OnFireCommand;
    std::function<void(const LaserControl&)>   OnLaserControl;
    std::function<void(const TrackTarget&)>    OnTrackTarget;
    std::function<void(const TrackRelease&)>   OnTrackRelease;
    std::function<void(const FireResult&)>     OnFireResult;
    std::function<void(const RobotMoving&)>    OnRobotMoving;
    std::function<void(const SoftReset&)>  OnSoftReset;

    std::function<void(bool)> ConnectionChanged; // 워치독

    explicit NetworkService(int localPort=50000)
        : udp_(localPort)
    {
        udp_.DataReceived = [this](const std::vector<uint8_t>& data, const std::string& ip, uint16_t port){
            (void)ip; (void)port;
            this->on_udp_received(data);
        };
    }

    bool start(){
        if (!udp_.start()) return false;
        watchdog_run_.store(true);
        //watchdog_thread_ = std::thread([this]{ this->watchdog_loop(); });
        std::cout << "[Network] UDP listening started.\n";
        return true;
    }
    void stop(){
        watchdog_run_.store(false);
        if (watchdog_thread_.joinable()) watchdog_thread_.join();
        udp_.stop();
        std::cout << "[Network] Stopped.\n";
    }
    ~NetworkService(){ stop(); }

    // ------------------ 업링크(Z->PC) 송신 헬퍼 (모두 UDP) ------------------
    bool send_status(const StatusData& s){
        Message m; m.type="status"; m.timestamp=iso8601_now(); m.data = s;
        return send_udp(m);
    }
    bool send_bbox(const BBoxData& b){
        Message m; m.type="bbox"; m.timestamp=iso8601_now(); m.data = b;
        return send_udp(m);
    }
    bool send_fire_ready(const FireReadyData& f){ // (변경) UDP 사용
        Message m; m.type="fire_ready"; m.timestamp=iso8601_now(); m.data = f;
        return send_udp(m);
    }
    bool send_fire_done(const FireDoneData& f){ // (변경) UDP 사용
        Message m; m.type="fire_done"; m.timestamp=iso8601_now(); m.data = f;
        return send_udp(m);
    }
    bool send_any(const Message& m){ return send_udp(m); }

private:
    UDPDataLink udp_;

    std::atomic<bool> watchdog_run_{false};
    std::thread watchdog_thread_;
    std::atomic<long long> last_heartbeat_ns_{0};
    std::atomic<bool> is_connected_{false};

    static inline long long now_ns(){
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }

    bool send_udp(const Message& m){
        try{
            nlohmann::json j = m;
            return udp_.send_text(j.dump());
        }catch(std::exception& e){
            std::cerr << "[UDP] send error: " << e.what() << "\n"; return false;
        }
    }

    void on_udp_received(const std::vector<uint8_t>& bytes){
        std::string s(bytes.begin(), bytes.end());
        try{
            Message msg = nlohmann::json::parse(s).get<Message>();
            // 하트비트(수신 시 접속 활성로 간주)
            last_heartbeat_ns_.store(now_ns(), std::memory_order_relaxed);

            const auto& t = msg.type;
            const auto& d = msg.data;
            std::cout << t << " " << d << "\n";
            if (t == "gimbal_control") {
                if (OnGimbalControl) OnGimbalControl(d.get<GimbalControl>());
            }
            else if (t == "fire_mode") {
                if (OnFireMode) OnFireMode(d.get<FireMode>());
            }
            else if (t == "operation_mode") {
                if (OnOperationMode) OnOperationMode(d.get<OperationMode>());
            }
            else if (t == "fire_command") {
                if (OnFireCommand) OnFireCommand(d.get<FireCommand>());
            }
            else if (t == "laser_control") {
                if (OnLaserControl) OnLaserControl(d.get<LaserControl>());
            }
            else if (t == "track_target") {
                if (OnTrackTarget) OnTrackTarget(d.get<TrackTarget>());
            }
            else if (t == "track_release") {
                if (OnTrackRelease) OnTrackRelease(TrackRelease{});
            }
            else if (t == "fire_result") {
                if (OnFireResult) OnFireResult(d.get<FireResult>());
            }
            else if (t == "robot_moving") {
                if (OnRobotMoving) OnRobotMoving(d.get<RobotMoving>());
            }
            else if (t == "soft_reset") {
                if (OnSoftReset) OnSoftReset(d.get<SoftReset>());
            }
            else {
                std::cout << "[UDP] Unknown type: " << t << "\n";
            }
        }catch(std::exception& e){
            std::cerr << "[UDP] parse error: " << e.what() << "\n";
        }
    }

    void watchdog_loop(){
        using namespace std::chrono_literals;
        while (watchdog_run_.load()){
            std::this_thread::sleep_for(1s);
            long long now = now_ns();
            long long last = last_heartbeat_ns_.load(std::memory_order_relaxed);
            bool connected = (last != 0) && ((now - last) <= 3'000'000'000LL);
            if (connected != is_connected_.load(std::memory_order_relaxed)){
                is_connected_.store(connected, std::memory_order_relaxed);
                if (ConnectionChanged) ConnectionChanged(connected);
                // std::cout << "[Network] Connection: " << (connected ? "Active" : "Lost") << "\n";
            }
        }
    }
};
