#pragma once
#include "network_service.hpp"
#include "network_types.hpp"
#include "vision_modules.hpp"

#include <stdexcept>
#include <algorithm>
#include <gst/gst.h>
#include <mutex>
#include <unordered_set>
#include <thread>
#include <chrono>

class Vision_App_Host {
private:
    std::atomic<bool> running{false};
    Video_Pipeline_Manager pipeline_mgr;
    Vision_Processor       processor;
    Vision_Infer           infer;
    Vision_Postproc        postproc;
    Object_Tracker         tracker;
    Target_Manager         target_mgr;
    NetworkService&        net;
    BBoxData& sharedBBox;
    std::mutex& sharedBBox_mtx;

    int bbox_centerX;
    int bbox_centerY;
    int bbox_width;
    int bbox_height;

    std::thread stream_thread;
    std::thread process_thread;

    static inline void upsert_by_id(BBoxData& dst, const BBoxObject& bo, size_t max_objs){
        auto& v = dst.objects;
        auto it = std::find_if(v.begin(), v.end(),
            [&](const BBoxObject& x){ return x.id == bo.id; });
        if (it != v.end()) {
            *it = bo;  // 같은 id면 값만 갱신
        } else if (v.size() < max_objs){
            v.push_back(bo); // 새 id면 추가
        } else {
            // 꽉 찼다면 정책 선택: 버리거나, 낮은 priority 교체 등
        }
    }

public:
    Vision_App_Host(const std::string& dev,
                    const std::string& pc_ip,     int video_port,
                    const std::string& meta_ip,   int meta_port,
                    const std::string& model_path,
                    NetworkService& net, BBoxData& bbox_data, std::mutex& shared_bbox_mutex_ref)
    : pipeline_mgr(dev, pc_ip, video_port),
      infer("EdgeTPU"),
      net(net),
      sharedBBox(bbox_data),
      sharedBBox_mtx(shared_bbox_mutex_ref)
    {
        // GStreamer 전역 초기화
        gst_init(nullptr, nullptr);

        // 1) 모델 로드 -> 2) 모델 입력 크기를 전처리에 전달
        if (!infer.load(model_path)) {
            throw std::runtime_error("[Application] Failed to load model: " + model_path);
        }
        processor.set_model_input_size(infer.input_w(), infer.input_h());

        {
            std::lock_guard<std::mutex> lk(sharedBBox_mtx);
            sharedBBox.objects.clear();         // 시작 시 빈 상태 권장
            sharedBBox.objects.reserve(10);     // MAX_OBJS 정도로 미리 예약
        }

        running.store(true);
        std::cout << "[Vision App Host] init OK. model=" << model_path
                  << " in=" << infer.input_w() << "x" << infer.input_h() << "\n";
    }

    ~Vision_App_Host() {
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

        // 비전 처리 스레드 (AI edgeTPU)
        process_thread = std::thread([this]{
            using clock = std::chrono::steady_clock;

            int frame_idx = 0;
            const float  SEND_CONF_TH = 0.69f;
            const size_t MAX_OBJS     = 5;

            // 5초 윈도우 성능 측정용
            auto   win_start    = clock::now();
            double sum_inf_ms   = 0.0;
            double sum_total_ms = 0.0;
            int    cnt_frames   = 0;

            while (running.load()) {
                long ts_ms = 0;

                auto t0 = clock::now();  // 한 프레임 전체 시작 시각

                cv::Mat frame = pipeline_mgr.pull_frame(ts_ms);
                if (frame.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                // 전처리
                int8_Tensor x = processor.preprocess(frame);

                // ===== inference 구간 시간 측정 =====
                auto t_inf0 = clock::now();
                Vision_RawOut r = infer.infer(x);
                auto t_inf1 = clock::now();
                // ===================================

                auto dets = postproc.run(r, "224x224"); //224x224

                // dt 계산 (ms)
                static long prev_ts = -1;
                float dt_ms = 33.0f;
                if (prev_ts >= 0 && ts_ms > 0) {
                    dt_ms = std::clamp(float(ts_ms - prev_ts), 10.0f, 100.0f);
                }
                prev_ts = ts_ms;

                // dt 추가된 트래킹
                dets = tracker.track(dets, dt_ms);

                const float SX = float(FRAME_WIDTH)  / float(infer.input_w());
                const float SY = float(FRAME_HEIGHT) / float(infer.input_h());

                // 이번 프레임에서 본 id들 기록
                std::unordered_set<int> seen;

                // ---- 공유 버퍼 갱신(업서트) ----
                BBoxData snapshot; // 전송용 스냅샷(락 바깥에서 보낼 것)
                {
                    std::lock_guard<std::mutex> lk(sharedBBox_mtx);

                    sharedBBox.frame_id = frame_idx++;

                    // 화면 중심 (640x480 기준)
                    constexpr int CENTER_X = FRAME_WIDTH  / 2;  // 320
                    constexpr int CENTER_Y = FRAME_HEIGHT / 2;  // 240

                    for (const auto& o : dets) {
                        if (o.confidence < SEND_CONF_TH) continue;

                        int cx = (int)std::lround(o.bbox[0] * SX);
                        int cy = (int)std::lround(o.bbox[1] * SY);
                        int ww = (int)std::lround(o.bbox[2] * SX);
                        int hh = (int)std::lround(o.bbox[3] * SY);

                        bbox_centerX = cx;
                        bbox_centerY = cy;
                        bbox_width   = ww;
                        bbox_height  = hh;

                        BBoxObject bo;
                        bo.id          = o.id;
                        bo.class_name  = o.obj_class;
                        bo.confidence  = o.confidence;
                        bo.bbox        = { cx, cy, ww, hh };

                        // 1단계: 우선 "거리 제곱"을 임시 priority로 저장
                        int dx = cx - CENTER_X;
                        int dy = cy - CENTER_Y;
                        bo.priority = dx*dx + dy*dy;   // 지금은 아직 '점수' 개념

                        upsert_by_id(sharedBBox, bo, MAX_OBJS);
                        seen.insert(bo.id);
                    }

                    // 이번 프레임에 보지 못한 id는 제거
                    auto& v = sharedBBox.objects;
                    v.erase(std::remove_if(v.begin(), v.end(),
                            [&](const BBoxObject& x){ return !seen.count(x.id); }),
                            v.end());

                    // 🔥 2단계: 거리 제곱(priority)에 따라 정렬 후, 1,2,3,... 순위로 다시 매기기
                    std::sort(v.begin(), v.end(),
                            [](const BBoxObject& a, const BBoxObject& b){
                                return a.priority < b.priority; // 거리 제곱이 작은 순
                            });

                    for (size_t i = 0; i < v.size(); ++i) {
                        v[i].priority = static_cast<int>(i) + 1;  // 1등, 2등, 3등...
                    }

                    snapshot = sharedBBox;
                }

                // ---- 락 해제 후 전송 ----
                if (!snapshot.objects.empty()){
                    net.send_bbox(snapshot);
                }

                // (짐벌 타겟 선택 로직 유지)
                if (!dets.empty()) {
                    auto target = target_mgr.getTrackedTarget(dets);
                    if (target.id != 0) {
                        // 추후 짐벌 제어용으로 사용
                    }
                }

                auto t_end = clock::now();  // 한 프레임 전체 끝 시각

                // ===== 5초 윈도우 성능 누적 =====
                double inf_ms   = std::chrono::duration<double, std::milli>(t_inf1 - t_inf0).count();
                double total_ms = std::chrono::duration<double, std::milli>(t_end   - t0).count();

                sum_inf_ms   += inf_ms;
                sum_total_ms += total_ms;
                cnt_frames++;

                if (t_end - win_start >= std::chrono::seconds(5)) {
                    if (cnt_frames > 0) {
                        double avg_inf   = sum_inf_ms   / cnt_frames;
                        double avg_total = sum_total_ms / cnt_frames;
                        double inf_fps   = 1000.0 / avg_inf;
                        double total_fps = 1000.0 / avg_total;

                        std::cout << "[PERF][5s] infer_avg=" << avg_inf   << " ms ("
                                  << inf_fps   << " FPS), "
                                  << "total_avg=" << avg_total << " ms ("
                                  << total_fps << " FPS) "
                                  << "frames=" << cnt_frames << "\n";
                    }

                    // 윈도우 리셋
                    win_start    = t_end;
                    sum_inf_ms   = 0.0;
                    sum_total_ms = 0.0;
                    cnt_frames   = 0;
                }
            }

            std::cout << "[T2] process loop exit\n";
        });
    }

    int getCenterX(){
        return bbox_centerX;
    }

    int getCenterY(){
        return bbox_centerY;
    }

    int getWidth(){
        return bbox_width;
    }

    int getHeight(){
        return bbox_height;
    }

    void request_stop() {
        stop();
    }

private:
    void stop() {
        if (!running.exchange(false)) return;
        pipeline_mgr.stop();

        if (stream_thread.joinable())  stream_thread.join();
        if (process_thread.joinable()) process_thread.join();
    }
};
