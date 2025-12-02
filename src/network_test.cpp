#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include "network_service.hpp"
#include "network_types.hpp"

// vision_main.cpp
#include "vision_modules.hpp"
#include "vision_data.h"
#include <gst/gst.h>

#include <opencv2/dnn.hpp>

#include <mutex>

class Application {
public:
    Application(const std::string& dev,
                const std::string& pc_ip,     int video_port,
                const std::string& meta_ip,   int meta_port,
                const std::string& model_path,
                NetworkService& net)
    : pipeline_mgr(dev, pc_ip, video_port),
      infer("EdgeTPU"),
      net(net)
    {
        // GStreamer 전역 초기화
        gst_init(nullptr, nullptr);

        // 1) 모델 로드 -> 2) 모델 입력 크기를 전처리에 전달
        if (!infer.load(model_path)) {
            throw std::runtime_error("[Application] Failed to load model: " + model_path);
        }
        processor.set_model_input_size(infer.input_w(), infer.input_h());

        running.store(true);
        std::cout << "[Application] init OK. model=" << model_path
                  << " in=" << infer.input_w() << "x" << infer.input_h() << "\n";
    }

    ~Application() {
        stop();
    }

    void run() {
        // 스트림 스레드 (GStreamer)
        stream_thread = std::thread([this]{
            if (!pipeline_mgr.run_pipeline_sync()) {
                std::cerr << "[T1] pipeline failed/EOS\n";
                running.store(false);
            }
        });

        // 비전 처리 스레드 (전처리 → 추론 → 후처리/트래킹 → 전송)
        process_thread = std::thread([this]{
            int frame_idx = 0;                         // ★ 프레임 카운터(이름 충돌 주의: cv::Mat frame과 다름)
            const float SEND_CONF_TH = 0.65f;          // 전송 임계값(옵션) 값 변경 필요 0.6->
            const size_t MAX_OBJS    = 10;             // 과대 패킷 방지 객체탐지 갯수

            while (running.load()) {
                long ts_ms = 0;
                cv::Mat frame = pipeline_mgr.pull_frame(ts_ms);
                if (frame.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                // 전처리 → 추론 → 후처리 → 트래킹 → 전송
                int8_Tensor x = processor.preprocess(frame);
                Vision_RawOut r = infer.infer(x);
                auto dets = postproc.run(r, "320x320"); // 640x480
                //dets = tracker.track(dets);

                const float SX = float(FRAME_WIDTH)  / float(infer.input_w());  // 640 / 224
                const float SY = float(FRAME_HEIGHT) / float(infer.input_h());  // 480 / 224

                BBoxData b;
                b.frame_id = frame_idx++;
                b.objects.reserve(std::min(dets.size(), MAX_OBJS));

                for (const auto& o : dets) {
                    if (o.confidence < SEND_CONF_TH) continue;

                    int cx = (int)std::lround(o.bbox[0] * SX);
                    int cy = (int)std::lround(o.bbox[1] * SY);
                    int ww = (int)std::lround(o.bbox[2] * SX);
                    int hh = (int)std::lround(o.bbox[3] * SY);

                    BBoxObject bo;
                    bo.id          = o.id;
                    bo.class_name  = o.obj_class;
                    bo.confidence  = o.confidence;
                    bo.priority    = o.priority;
                    // ★ 최종 전송 포맷: (center_x, center_y, width, height)
                    bo.bbox = { cx, cy, ww, hh };

                    b.objects.push_back(std::move(bo));
                    if (b.objects.size() >= MAX_OBJS) break;
                }

                // ★ 바로 송신
                if (!b.objects.empty()){
                    net.send_bbox(b);
                    std::cout << "[BBOX→PC] frame="<<b.frame_id<<" objs="<<b.objects.size()<<"\n";
                }



                // (짐벌 전송 로직은 그대로 유지)
                if (!dets.empty()) {
                    auto target = target_mgr.getTrackedTarget(dets);
                    if (target.id != 0) {
                        //int* g = target_mgr.getGimbalCoords(target);
                    }
                }
            }
            std::cout << "[T2] process loop exit\n";
        });

        // 종료 대기(ENTER)
        std::cin.get();
        while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        stop();
    }

private:
    void stop() {
        if (!running.exchange(false)) return;
        pipeline_mgr.stop();

        if (stream_thread.joinable())  stream_thread.join();
        if (process_thread.joinable()) process_thread.join();
    }

private:
    std::atomic<bool> running{false};
    Video_Pipeline_Manager pipeline_mgr;
    Vision_Processor       processor;
    Vision_Infer           infer;
    Vision_Postproc        postproc;
    Object_Tracker         tracker;
    Target_Manager         target_mgr;
    NetworkService&        net;

    std::thread stream_thread;
    std::thread process_thread;
};


static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run.store(false); }

int main(){
    std::signal(SIGINT, on_sigint);

    const std::string DEV        = "/dev/video0";
    const std::string PC_IP      = "192.168.137.1";   // JPEG RTP 보내는 대상
    const int         VIDEO_PORT = 5600;

    const std::string META_IP    = "192.168.137.1";   // 메타데이터 수신 PC
    const int         META_PORT  = 5000;

    const std::string MODEL      = "/usr/local/share/models/v8_320_best_full_integer_quant_edgetpu.tflite";

    NetworkService net(5000);

    std::cout << "Test" << "\n";

    // 수신(PC->Zynq) 콜백들
    net.OnGimbalControl = [](const GimbalControl& v){
        std::cout << "[RX] gimbal_control yaw=" << v.yaw << " pitch=" << v.pitch << "\n";
    };
    net.OnFireMode = [](const FireMode& v){
        std::cout << "[RX] fire_mode mode=" << v.mode << "\n";
    };
    net.OnOperationMode = [](const OperationMode& v){
        std::cout << "[RX] operation_mode mode=" << v.mode << "\n";
    };
    net.OnFireCommand = [](const FireCommand& v){
        std::cout << "[RX] fire_command trigger=" << (v.trigger?"true":"false") << "\n";
    };
    net.OnLaserControl = [](const LaserControl& v){
        std::cout << "[RX] laser_control state=" << v.state << "\n";
    };
    net.OnTrackTarget = [](const TrackTarget& v){
        std::cout << "[RX] track_target id=" << v.target_id << "\n";
    };
    net.OnTrackRelease = [](const TrackRelease&){
        std::cout << "[RX] track_release\n";
    };
    net.OnFireResult = [](const FireResult& v){
        std::cout << "[RX] fire_result success=" << (v.success?"true":"false") << "\n";
    };
    net.OnRobotMoving = [](const RobotMoving& v){
        std::cout << "[RX] robot_moving moving=" << v.moving << "\n";
    };
    net.OnSoftReset = [](const SoftReset& v){
        std::cout << "[RX] soft_reset reset=" << (v.reset?"true":"false") << "\n";
    };
    net.ConnectionChanged = [](bool on){
        std::cout << "[NET] " << (on ? "Active" : "Lost") << "\n";
    };

    if (!net.start()){
        std::cerr << "network start failed\n";
        return 1;
    }

    std::atomic<bool> app_running{true};

    // Application 객체를 스레드에서 실행
    std::thread app_thread([&]{
        try {
            Application app(DEV, PC_IP, VIDEO_PORT, META_IP, META_PORT, MODEL, net); // 마지막 인자로 net도 전달
            app.run(); // 이 안에서 stream/process 스레드가 돌아감
        } catch (const std::exception& e) {
            std::cerr << "fatal: " << e.what() << "\n";
            app_running.store(false);
        }
    });


    // --- 데모: 100ms status, 33ms bbox 주기 송신 ---
    std::thread th_status([&]{
        using namespace std::chrono;
        while (g_run.load()){
            StatusData s;
            s.yaw = 48.6;
            net.send_status(s);
            std::this_thread::sleep_for(100ms);
        }
    });
/*
    std::thread th_bbox([&]{
        using namespace std::chrono;
        int frame=0;
        while (g_run.load()){
            BBoxData b;
            b.frame_id = frame++;
            b.objects.push_back(BBoxObject{1,"target",0.92,{100,120,200,240}, 1});
            b.objects.push_back(BBoxObject{2,"target",0.92,{200,100,300,320}, 2});
            net.send_bbox(b);
            std::this_thread::sleep_for(33ms); // ~30 FPS
        }
    });
*/
    // 데모: 5초 후 fire_ready(UDP) 한 번 송신
    std::this_thread::sleep_for(std::chrono::seconds(5));
    net.send_fire_ready(FireReadyData{1, true});

    std::cout << "Press Ctrl+C to quit\n";
    while (g_run.load()) std::this_thread::sleep_for(std::chrono::seconds(1));

    // 종료 정리
    //if (th_status.joinable()) th_status.join();
    //if (th_bbox.joinable())   th_bbox.join();
    net.stop();
    return 0;
}