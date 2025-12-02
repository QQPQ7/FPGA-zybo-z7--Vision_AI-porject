// vision_main.cpp
#include "vision_modules.hpp"
#include "vision_data.h"
#include <gst/gst.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <chrono>

class Application {
public:
    Application(const std::string& dev,
                const std::string& pc_ip,     int video_port,
                const std::string& meta_ip,   int meta_port,
                const std::string& model_path)
    : pipeline_mgr(dev, pc_ip, video_port),
      infer("EdgeTPU")
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
                auto dets = postproc.run(r, "640x480");
                //dets = tracker.track(dets);

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

    std::thread stream_thread;
    std::thread process_thread;
};

int main(int argc, char** argv) {
    // 필요시 CLI 인자로 바꿔도 됨
    const std::string DEV        = "/dev/video0";
    const std::string PC_IP      = "192.168.2.10";   // JPEG RTP 보내는 대상
    const int         VIDEO_PORT = 5000;

    const std::string META_IP    = "192.168.1.10";   // 메타데이터 수신 PC
    const int         META_PORT  = 5001;

    const std::string MODEL      = "/usr/local/share/models/yolov8n_full_integer_quant_edgetpu.tflite";

    try {
        Application app(DEV, PC_IP, VIDEO_PORT, META_IP, META_PORT, MODEL);
        app.run();
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    }
    return 0;
}

