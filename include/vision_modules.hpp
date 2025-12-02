#pragma once
#include "vision_data.h"

#include <iostream>
#include <stdexcept>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <sstream>
#include <iomanip>
#include <memory>

#include <unordered_map>
#include <map>
#include <glib.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/video/tracking.hpp>  // KalmanFilter

// POSIX socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// GStreamer
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

// TensorFlow Lite + External delegate
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
//#include <tensorflow/lite/delegates/external/external_delegate.h>
#include <dlfcn.h>  // ✅ dlopen, dlsym, dlclose

#include <unordered_set>
#include <numeric>   // std::iota


struct TfLiteDelegate;

// -----------------------------
// bytetrack-lite
// -----------------------------
class ByteTrackLite {
public:
    struct Params {
        float high_th = 0.50f;   // 1차 매칭용 confidence
        float low_th  = 0.20f;   // 2차 매칭용 confidence
        float iou_high = 0.40f;  // 1차 IoU 임계
        float iou_low  = 0.30f;  // 2차 IoU 임계
        int   max_age  = 8;     // 실종 허용 프레임
        int   min_hits = 3;      // 확정까지 필요한 히트 수
    } p;

    std::vector<DetectedObject> update(const std::vector<DetectedObject>& dets);

private:
    struct Track {
        int id;
        int bbox[4];     // CXYWH(int)
        float score;
        int age;         // miss 누적
        int hits;        // 연속 hit
        bool confirmed;  // hits >= min_hits
        std::string cls;
        int priority;
    };
    int next_id = 1;
    std::vector<Track> tracks;

    static float iou_cxywh(const int a[4], const int b[4]) {
        auto to_xyxy = [](const int t[4]){
            int x1 = t[0] - t[2]/2, y1 = t[1] - t[3]/2;
            int x2 = x1 + t[2],     y2 = y1 + t[3];
            return std::array<int,4>{x1,y1,x2,y2};
        };
        auto A = to_xyxy(a), B = to_xyxy(b);
        int ix0 = std::max(A[0], B[0]), iy0 = std::max(A[1], B[1]);
        int ix1 = std::min(A[2], B[2]), iy1 = std::min(A[3], B[3]);
        int iw = std::max(0, ix1-ix0), ih = std::max(0, iy1-iy0);
        long inter = 1L*iw*ih;
        long areaA = 1L*(A[2]-A[0])*(A[3]-A[1]);
        long areaB = 1L*(B[2]-B[0])*(B[3]-B[1]);
        long uni = areaA + areaB - inter;
        return (uni>0) ? float(inter)/float(uni) : 0.f;
    }

    // 간단 Greedy 매칭: IoU 내림차순으로 매칭
    static void greedy_assign(
        const std::vector<int>& t_idx,
        const std::vector<int>& d_idx,
        const std::vector<Track>& tracks,
        const std::vector<DetectedObject>& dets,
        float iou_th,
        std::vector<std::pair<int,int>>& matches,
        std::vector<int>& un_trk,
        std::vector<int>& un_det)
    {
        struct Cand{int ti, di; float iou;};
        std::vector<Cand> cands; cands.reserve(t_idx.size()*d_idx.size());
        for (int ti : t_idx){
            for (int di : d_idx){
                float v = iou_cxywh(tracks[ti].bbox, dets[di].bbox);
                if (v >= iou_th) cands.push_back({ti,di,v});
            }
        }
        std::sort(cands.begin(), cands.end(), [](auto& a, auto& b){ return a.iou>b.iou; });

        std::vector<bool> used_t(tracks.size(), false), used_d(dets.size(), false);
        for (auto& c : cands){
            if (!used_t[c.ti] && !used_d[c.di]){
                used_t[c.ti] = used_d[c.di] = true;
                matches.push_back({c.ti, c.di});
            }
        }
        for (int ti : t_idx) if (!used_t[ti]) un_trk.push_back(ti);
        for (int di : d_idx) if (!used_d[di]) un_det.push_back(di);
    }
};

// ---- 구현부(헤더 온리) ----
inline std::vector<DetectedObject>
ByteTrackLite::update(const std::vector<DetectedObject>& dets) {
    // 1) H / L 분리
    std::vector<int> H, L;
    for (int i=0;i<(int)dets.size();++i){
        if (dets[i].confidence >= p.high_th) H.push_back(i);
        else if (dets[i].confidence >= p.low_th) L.push_back(i);
    }

    // 현재 트랙 인덱스
    std::vector<int> T(tracks.size()); std::iota(T.begin(), T.end(), 0);

    // 2) 1차 매칭 (H)
    std::vector<std::pair<int,int>> m1;
    std::vector<int> unT1, unH;
    greedy_assign(T, H, tracks, dets, p.iou_high, m1, unT1, unH);

    // 3) 트랙 업데이트 (matched H)
    std::vector<bool> det_used(dets.size(), false);
    for (auto& [ti, di] : m1){
        det_used[di] = true;
        auto& tr = tracks[ti];
        for (int k=0;k<4;++k) tr.bbox[k] = dets[di].bbox[k];
        tr.score = dets[di].confidence;
        tr.cls   = dets[di].obj_class;
        tr.priority = dets[di].priority;
        tr.age = 0; tr.hits++;
        if (!tr.confirmed && tr.hits >= p.min_hits) tr.confirmed = true;
    }

    // 4) 2차 매칭 (남은 트랙 ↔ L)
    std::vector<std::pair<int,int>> m2;
    std::vector<int> unT2, unL;
    greedy_assign(unT1, L, tracks, dets, p.iou_low, m2, unT2, unL);
    for (auto& [ti, di] : m2){
        det_used[di] = true;
        auto& tr = tracks[ti];
        for (int k=0;k<4;++k) tr.bbox[k] = dets[di].bbox[k];
        tr.score = dets[di].confidence;
        tr.cls   = dets[di].obj_class;
        tr.priority = dets[di].priority;
        tr.age = 0; tr.hits++;
        if (!tr.confirmed && tr.hits >= p.min_hits) tr.confirmed = true;
    }

    // 5) 새 트랙 생성 (unmatched H만)
    for (int di : unH){
        Track t{};
        t.id = next_id++;
        for (int k=0;k<4;++k) t.bbox[k] = dets[di].bbox[k];
        t.score = dets[di].confidence;
        t.cls   = dets[di].obj_class;
        t.priority = dets[di].priority;
        t.age = 0; t.hits = 1; t.confirmed = (p.min_hits<=1);
        tracks.push_back(t);
    }

    // 6) 실종 증가 및 정리
    std::vector<Track> kept; kept.reserve(tracks.size());
    for (auto& tr : tracks){
        bool matched = false;
        for (auto& pr : m1) if (tracks[pr.first].id==tr.id) { matched=true; break; }
        if (!matched) for (auto& pr : m2) if (tracks[pr.first].id==tr.id) { matched=true; break; }
        if (!matched) tr.age++;

        if (tr.age <= p.max_age) kept.push_back(tr);
    }
    tracks.swap(kept);

    // 7) 출력(확정 트랙만 내보내고, det에 id 매핑)
    //    (원하면 미확정도 내보낼 수 있음)
    std::vector<DetectedObject> out; out.reserve(dets.size());
    for (int di=0; di<(int)dets.size(); ++di){
        int assign_id = 0;
        // m1/m2에서 di를 찾기
        for (auto& [ti, dj] : m1) if (dj==di) { assign_id = tracks[ti].id; break; }
        if (!assign_id) for (auto& [ti, dj] : m2) if (dj==di) { assign_id = tracks[ti].id; break; }

        if (assign_id){
            DetectedObject o = dets[di];
            o.id = assign_id;
            out.push_back(std::move(o));
        }
    }
    return out;
}

// -----------------------------
// CXY2TLWH (for NMS)
// -----------------------------
inline void set_cxywh(DetectedObject& o, float cx, float cy, float w, float h){
    o.bbox[0] = (int)std::lround(cx);
    o.bbox[1] = (int)std::lround(cy);
    o.bbox[2] = (int)std::lround(w);
    o.bbox[3] = (int)std::lround(h);
}

inline std::array<float,4> get_cxywhf(const DetectedObject& o){
    return { (float)o.bbox[0], (float)o.bbox[1], (float)o.bbox[2], (float)o.bbox[3] };

}

// NMSBoxes용 일시 변환: CXYWH(int) → TLWH(int)
inline cv::Rect cxywh_to_tlwh(const DetectedObject& o){
    int x = o.bbox[0] - o.bbox[2]/2;
    int y = o.bbox[1] - o.bbox[3]/2;
    return { x, y, o.bbox[2], o.bbox[3] };
}

// -----------------------------
// GStreamer init (once)
// -----------------------------
static bool s_gst_inited = false;

void init_gstreamer_if_needed() {
    if (!s_gst_inited) {
        if (!gst_is_initialized()) {
            if (!gst_init_check(nullptr, nullptr, nullptr)) {
                std::cerr << "[GStreamer] init failed\n";
                std::exit(1);
            }
        }
        s_gst_inited = true;
        std::cout << "[GStreamer] Initialized.\n";
    }
}

// =====================================================================
// A. Video_Pipeline_Manager (GStreamer를 통한 비디오 캡처 및 스트리밍)
// =====================================================================
class Video_Pipeline_Manager {
private:
    const std::string V4L2_DEVICE;
    const std::string HOST_PC_IP;
    const int VIDEO_PORT;

    GstElement *pipeline = nullptr;
    
    // GStreamer 콜백에서 접근하기 위한 공유 데이터 구조체
    struct SharedGstData {
        GstSample *sample = nullptr;
        long timestamp_ms = 0;
        gboolean running = TRUE; // 파이프라인 실행 상태
        GMutex mutex;
        GCond cond;
    } shared_data;

    std::string build_pipeline_string() const {
        const int W = FRAME_WIDTH, H = FRAME_HEIGHT, FPS = FRAME_RATE;

        std::string caps = "video/x-raw,format=YUY2,width=" + std::to_string(W) +
                           ",height=" + std::to_string(H) +
                           ",framerate=" + std::to_string(FPS) + "/1";

        // tee → appsink(appsink) for AI, and jpeg → rtp → udpsink for video streaming
        // std::string p =
        //     "v4l2src device=" + V4L2_DEVICE + " ! " +
        //      caps + " ! videoconvert ! opencvvideostab ! videoconvert ! tee name=t "
        //     "t. ! queue max-size-buffers=1 leaky=downstream ! appsink name=appsink drop=true sync=false "
        //     "t. ! queue ! jpegenc quality=50 ! rtpjpegpay ! udpsink host=" + HOST_PC_IP +
        //     " port=" + std::to_string(VIDEO_PORT);

                    std::string p =
            "v4l2src device=" + V4L2_DEVICE + " ! " +
            caps + " ! tee name=t "
            // AI 브랜치
            "t. ! queue max-size-buffers=1 leaky=downstream ! appsink name=appsink drop=true sync=false "

            // H.264 스트리밍 브랜치
            "t. ! queue max-size-buffers=1 leaky=downstream  ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=500 key-int-max=15 bframes=0 ! "
            "video/x-h264,profile=baseline ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host=" + HOST_PC_IP +
            " port=" + std::to_string(VIDEO_PORT) +
            " sync=false async=false";
        return p;

    }

    cv::Mat gst_sample_to_mat(GstSample *sample) {
        if (!sample) return {};

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) return {};

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) return {};

        // --- caps에서 width/height/format 읽기 ---
        int width = FRAME_WIDTH, height = FRAME_HEIGHT;
        const gchar* fmt = nullptr;
        if (GstCaps* caps = gst_sample_get_caps(sample)) {
            if (GstStructure* st = gst_caps_get_structure(caps, 0)) {
                gst_structure_get_int(st, "width", &width);
                gst_structure_get_int(st, "height", &height);
                fmt = gst_structure_get_string(st, "format");
            }
        }

        cv::Mat bgr; // 결과
        if (fmt && g_str_equal(fmt, "I420")) {
            const size_t need = (size_t)width * height * 3 / 2;
            if (map.size < need) { // 짧은 프레임 가드
                gst_buffer_unmap(buffer, &map);
                return {};
            }
            // I420는 H*1.5 x W, 1채널 버퍼
            cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<guint8*>(map.data));
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
        } else if (fmt && g_str_equal(fmt, "YUY2")) {
            const size_t need = (size_t)width * height * 2;
            if (map.size < need) {
                gst_buffer_unmap(buffer, &map);
                return {};
            }
            cv::Mat yuy2(height, width, CV_8UC2, const_cast<guint8*>(map.data));
            cv::cvtColor(yuy2, bgr, cv::COLOR_YUV2BGR_YUY2);
        } else {
            // 예상치 못한 포맷이면 스킵
            gst_buffer_unmap(buffer, &map);
            return {};
        }

        gst_buffer_unmap(buffer, &map);
        return bgr; // cvtColor가 새 버퍼를 만들어주므로 수명 안전
    }

    // GstAppSink 콜백 함수 (static)
    static GstFlowReturn on_new_sample_from_sink(GstAppSink *sink, gpointer user_data) {
        auto* self = static_cast<Video_Pipeline_Manager*>(user_data);

        // 50ms 타임아웃으로 NULL 방지 (OK)
        GstSample *sample = gst_app_sink_try_pull_sample(sink, 50 * GST_MSECOND);
        if (!sample) return GST_FLOW_OK; // 프레임 없으면 그냥 패스

        g_mutex_lock(&self->shared_data.mutex);
        
        // 처리 스레드가 아직 이전 프레임을 가져가지 않았다면 (self->shared_data.sample != nullptr)
        // 새 프레임을 버리고 이전 프레임을 유지합니다.
        if (self->shared_data.sample) {
            gst_sample_unref(sample); // 새 프레임 버림
        } else {
            // 이전 프레임이 처리되었다면 (nullptr), 새 프레임으로 갱신
            self->shared_data.sample = sample;
            if (GstBuffer* buf = gst_sample_get_buffer(sample)) {
                self->shared_data.timestamp_ms = GST_BUFFER_PTS_IS_VALID(buf)
                    ? (GST_BUFFER_PTS(buf) / GST_MSECOND) : 0;
            }
            // g_cond_signal(&self->shared_data.cond); // pull_frame이 non-blocking이므로 signal은 필요 없음
        }

        g_mutex_unlock(&self->shared_data.mutex);
        return GST_FLOW_OK;
    }

public:
    Video_Pipeline_Manager(const std::string& device_path,
                                               const std::string& host_ip,
                                               int port)
    : V4L2_DEVICE(device_path), HOST_PC_IP(host_ip), VIDEO_PORT(port)
    {
        init_gstreamer_if_needed();
        g_mutex_init(&shared_data.mutex);
        g_cond_init(&shared_data.cond);
        std::cout << "[Pipeline] device=" << V4L2_DEVICE
                  << " stream-> " << HOST_PC_IP << ":" << VIDEO_PORT << "\n";
    }

    ~Video_Pipeline_Manager() {
        stop();
        g_mutex_clear(&shared_data.mutex);
        g_cond_clear(&shared_data.cond);
    }

    // 동기식으로 파이프라인을 실행하고 버스 메시지를 블록킹하며 대기
    bool run_pipeline_sync() {
        std::string desc = build_pipeline_string();
        std::cout << "[GStreamer] Pipeline:\n" << desc << "\n";

        GError* err = nullptr;
        pipeline = gst_parse_launch(desc.c_str(), &err);
        if (!pipeline) {
            std::cerr << "[GStreamer] parse_launch failed: "
                      << (err? err->message : "unknown") << "\n";
        if (err) g_error_free(err);
            return false;
        }

        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
        if (!sink) {
            std::cerr << "[GStreamer] appsink 'appsink' not found\n";
            gst_object_unref(pipeline); pipeline=nullptr;
            return false;
        }

        GstAppSinkCallbacks cbs { nullptr, nullptr, on_new_sample_from_sink };
        gst_app_sink_set_callbacks(GST_APP_SINK(sink), &cbs, this, nullptr);
        gst_object_unref(sink);

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "[GStreamer] set_state(PLAYING) failed\n";
            gst_object_unref(pipeline); pipeline=nullptr;
            return false;
        }

        // Block until ERROR/EOS
        GstBus *bus = gst_element_get_bus(pipeline);
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus, GST_CLOCK_TIME_NONE, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        bool ok = true;
        if (msg) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR:{
                    GError* e=nullptr; gchar* dbg=nullptr;
                    gst_message_parse_error(msg, &e, &dbg);
                    std::cerr << "[GStreamer] ERROR: " << (e? e->message:"") << "\n";
                    if (dbg) g_free(dbg);
                    if (e) g_clear_error(&e);
                    ok = false;
                } break;
                case GST_MESSAGE_EOS:
                    std::cout << "[GStreamer] EOS\n"; break;
                default: break;
            }
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
        return ok;
    }

    // AI 스레드에서 최신 프레임을 가져오는 함수
    cv::Mat pull_frame(long& timestamp_ms) {
        cv::Mat frame;

        g_mutex_lock(&shared_data.mutex);
        if (!shared_data.sample) {
            g_mutex_unlock(&shared_data.mutex);
            return {}; // non-blocking: 없으면 빈 Mat
        }
        GstSample* sample = shared_data.sample;
        shared_data.sample = nullptr;
        timestamp_ms = shared_data.timestamp_ms;
        g_mutex_unlock(&shared_data.mutex);

        frame = gst_sample_to_mat(sample);
        gst_sample_unref(sample);
        return frame;
    }

    void stop() {
        if (pipeline) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline=nullptr;
        }
        g_mutex_lock(&shared_data.mutex);
        shared_data.running = FALSE;
        g_cond_broadcast(&shared_data.cond);
        g_mutex_unlock(&shared_data.mutex);
        std::cout << "[Pipeline] stopped.\n";
    }
};


// =====================================================================
// B. Vision_Processor
// =====================================================================
class Vision_Processor {
public:
    void set_model_input_size(int w, int h) {
        if (w>0 && h>0){ target_w_ = w; target_h_ = h; }
    }

    int8_Tensor preprocess(const cv::Mat& input_mat) {
        int8_Tensor out;
        if (input_mat.empty()){
            std::cerr << "[Processor] empty frame\n";
            return out;
        }
        cv::Mat resized, rgb;
        cv::resize(input_mat, resized, cv::Size(target_w_, target_h_), 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

        out.shape = {1, target_h_, target_w_, 3};
        const size_t bytes = static_cast<size_t>(target_w_) * target_h_ * 3;
        out.data.resize(bytes);
        std::memcpy(out.data.data(), rgb.data, bytes); // 0..255 as int8 storage
        return out;
    }

private:
    int target_w_ = 224;
    int target_h_ = 224;
};

// =====================================================================
// C. Vision_Infer (TFLite + EdgeTPU External Delegate)
// =====================================================================
// TfLiteDelegate 전방 선언(혹은 <tensorflow/lite/c/common.h> 포함해도 됨)

class Vision_Infer {
public:
    // ✅ Vision_App_Host에서 infer("EdgeTPU")로 부를 수 있게 기본값 포함
    explicit Vision_Infer(const std::string& backend = "EdgeTPU") {}

    ~Vision_Infer(){
        // 1) Interpreter 파괴
        interp_.reset();

        // 2) Delegate 파괴
        if (delegate_ && destroy_fn_) { destroy_fn_(delegate_); delegate_ = nullptr; }
        // 3) libedgetpu 핸들 닫기
        if (edgetpu_lib_) { dlclose(edgetpu_lib_); edgetpu_lib_ = nullptr; }
    }

    bool load(const std::string& modelPath){
        model_ = tflite::FlatBufferModel::BuildFromFile(modelPath.c_str());
        if (!model_) { std::cerr << "[Infer] model open fail: " << modelPath << "\n"; return false; }

        tflite::ops::builtin::BuiltinOpResolver resolver;
        tflite::InterpreterBuilder builder(*model_, resolver);
        if (builder(&interp_) != kTfLiteOk || !interp_) {
            std::cerr << "[Infer] InterpreterBuilder failed\n"; return false;
        }

        // 🔁 외부 delegate 헬퍼 제거하고, 우리가 만든 멤버 함수 호출
        if (!load_edgetpu_delegate("usb:0")) {
            std::cerr << "[Infer] load_edgetpu_delegate failed\n"; return false;
        }
        if (interp_->ModifyGraphWithDelegate(delegate_) != kTfLiteOk) {
            std::cerr << "[Infer] ModifyGraphWithDelegate failed\n"; return false;
        }

        interp_->SetNumThreads(1);
        if (interp_->AllocateTensors() != kTfLiteOk) {
            std::cerr << "[Infer] AllocateTensors failed\n"; return false;
        }

        TfLiteTensor* in = interp_->input_tensor(0);
        if (!in || !in->dims || in->dims->size < 4) {
            std::cerr << "[Infer] invalid input dims\n"; return false;
        }
        in_h_ = in->dims->data[1];
        in_w_ = in->dims->data[2];
        in_c_ = in->dims->data[3];

        std::cout << "[Infer] loaded: " << modelPath
                  << " input=" << in_w_ << "x" << in_h_ << "x" << in_c_ << "\n";
        return true;
    }

    Vision_RawOut infer(const int8_Tensor& x){
        Vision_RawOut out;
        if (!interp_) { std::cerr << "[Infer] not loaded\n"; return out; }
        if (x.shape.size()!=4 || x.shape[1]!=in_h_ || x.shape[2]!=in_w_ || x.shape[3]!=3){
            std::cerr << "[Infer] size mismatch. expected "
                      << in_w_ << "x" << in_h_ << ", got "
                      << x.shape[2] << "x" << x.shape[1] << "\n";
            return out;
        }

        TfLiteTensor* in = interp_->input_tensor(0);
        const size_t Nbytes = static_cast<size_t>(in_w_) * in_h_ * 3;

        if (in->type == kTfLiteUInt8){
            uint8_t* dst = interp_->typed_input_tensor<uint8_t>(0);
            std::memcpy(dst, reinterpret_cast<const uint8_t*>(x.data.data()), Nbytes);
        } else if (in->type == kTfLiteInt8){
            const float scale = in->params.scale ? in->params.scale : (1.f/255.f);
            const int zp = in->params.zero_point;
            int8_t* dst = interp_->typed_input_tensor<int8_t>(0);
            const uint8_t* src = reinterpret_cast<const uint8_t*>(x.data.data());
            for (size_t i=0;i<Nbytes;++i){
                float xf = src[i] / 255.f;
                dst[i] = quantize_f32_to_i8(xf, scale, zp);
            }
        } else if (in->type == kTfLiteFloat32){
            float* dst = interp_->typed_input_tensor<float>(0);
            const uint8_t* src = reinterpret_cast<const uint8_t*>(x.data.data());
            for (size_t i=0;i<Nbytes;++i) dst[i] = src[i] / 255.f;
        } else {
            std::cerr << "[Infer] unsupported input type: " << in->type << "\n";
            return out;
        }

        if (interp_->Invoke() != kTfLiteOk){
            std::cerr << "[Infer] Invoke failed\n"; return out;
        }

        const TfLiteTensor* o = interp_->output_tensor(0);
        if (!o || !o->dims){ std::cerr << "[Infer] no output\n"; return out; }

        auto* dims = o->dims;
        const int rank = dims->size;
        const float conf_th = 0.62f; // 탐지 정확도 스레시홀드

        auto push_box = [&](float cx, float cy, float w, float h, float score, int cls){
            if (std::max(std::max(cx,cy), std::max(w,h)) <= 1.f){
                cx *= in_w_; cy *= in_h_; w *= in_w_; h *= in_h_;
            }
            float x1 = cx - w*0.5f, y1 = cy - h*0.5f;
            float x2 = cx + w*0.5f, y2 = cy + h*0.5f;
            x1 = std::max(0.f, std::min(x1, (float)in_w_-1));
            y1 = std::max(0.f, std::min(y1, (float)in_h_-1));
            x2 = std::max(0.f, std::min(x2, (float)in_w_-1));
            y2 = std::max(0.f, std::min(y2, (float)in_h_-1));
            out.boxes.push_back({x1,y1,x2,y2});
            out.scores.push_back(score);
            out.labels.push_back(cls);
        };

        auto sigmoid = [](float x){
            if (x>=0) return 1.f/(1.f+std::exp(-x));
            float e = std::exp(x); return e/(1.f+e);
        };

        std::vector<float> of; of.reserve(o->bytes/sizeof(float));
        if (o->type == kTfLiteFloat32){
            const float* f = o->data.f;
            of.assign(f, f + (o->bytes/sizeof(float)));
        } else if (o->type == kTfLiteInt8){
            const int8_t* q = o->data.int8;
            const float sc = o->params.scale;
            const int zp = o->params.zero_point;
            of.resize(o->bytes);
            for (int i=0;i<o->bytes;++i) of[i] = dequantize_i8_to_f32(q[i], sc, zp);
        } else {
            std::cerr << "[Infer] unsupported output type: " << o->type << "\n";
            return out;
        }

        if (rank == 3 && dims->data[0] == 1 && dims->data[1] >= 5) {
            const int C  = dims->data[1];      // 4 + nc
            const int N  = dims->data[2];
            const int NC = C - 4;              // 클래스 개수

            for (int i = 0; i < N; ++i) {
                float cx = of[0 * N + i];
                float cy = of[1 * N + i];
                float w  = of[2 * N + i];
                float h  = of[3 * N + i];

                int   best_cls = -1;
                float best_p   = 0.f;

                for (int c = 0; c < NC; ++c) {
                    float raw = of[(4 + c) * N + i];

                    
                    float p = raw;
                    // float p = std::clamp(raw, 0.0f, 1.0f);

                    if (p > best_p) {
                        best_p   = p;
                        best_cls = c;
                    }
                }

                if (best_p >= conf_th)  push_box(cx, cy, w, h, best_p, best_cls);
            }
        } else if (rank==3 && dims->data[0]==1 && dims->data[2]==6){
            // (1,N,6): [x1,y1,x2,y2,score,cls]
            const int N = dims->data[1];
            for (int i=0;i<N;++i){
                float x1 = of[i*6+0], y1 = of[i*6+1];
                float x2 = of[i*6+2], y2 = of[i*6+3];
                float sc = of[i*6+4];
                int cls  = (int)std::round(of[i*6+5]);
                if (sc>=conf_th){
                    if (std::max(std::max(x1,y1), std::max(x2,y2)) <= 1.f){
                        x1*=in_w_; x2*=in_w_; y1*=in_h_; y2*=in_h_;
                    }
                    out.boxes.push_back({x1,y1,x2,y2});
                    out.scores.push_back(sc);
                    out.labels.push_back(cls);
                }
            }
        } else {
            std::cerr << "[Infer] unknown output shape (rank="<<rank<<")\n";
        }
        return out;
    }

    // ✅ 인풋 사이즈 접근자
    int input_w() const { return in_w_; }
    int input_h() const { return in_h_; }

    static inline int8_t quantize_f32_to_i8(float x, float scale, int zp){
        int v = static_cast<int>(std::round(x/scale + zp));
        if (v>127) v=127; if (v<-128) v=-128;
        return static_cast<int8_t>(v);
    }
    static inline float dequantize_i8_to_f32(int8_t q, float scale, int zp){
        return (static_cast<int>(q) - zp) * scale;
    }

private:
    std::string backend;

    // TFLite 핸들들
    std::unique_ptr<tflite::FlatBufferModel> model_;
    std::unique_ptr<tflite::Interpreter>     interp_;
    int in_w_ = 0, in_h_ = 0, in_c_ = 0;

    // dlopen
    void* edgetpu_lib_ = nullptr;
    using CreateFn  = TfLiteDelegate* (*)(const char* const*, const char* const*, int, char**);
    using DestroyFn = void (*)(TfLiteDelegate*);
    CreateFn  create_fn_  = nullptr;
    DestroyFn destroy_fn_ = nullptr;
    TfLiteDelegate* delegate_ = nullptr;

    bool load_edgetpu_delegate(const char* device) {
        edgetpu_lib_ = dlopen("libedgetpu.so.1", RTLD_NOW | RTLD_LOCAL);
        if (!edgetpu_lib_) {
            std::cerr << "[Infer] dlopen libedgetpu.so.1 failed: " << dlerror() << "\n";
            return false;
        }
        create_fn_  = reinterpret_cast<CreateFn>( dlsym(edgetpu_lib_, "tflite_plugin_create_delegate") );
        destroy_fn_ = reinterpret_cast<DestroyFn>(dlsym(edgetpu_lib_, "tflite_plugin_destroy_delegate"));
        if (!create_fn_ || !destroy_fn_) {
            std::cerr << "[Infer] dlsym for tflite_plugin_* failed.\n";
            dlclose(edgetpu_lib_); edgetpu_lib_ = nullptr;
            return false;
        }
        const char* keys[]   = {"device"};
        const char* values[] = { device ? device : "usb:0" };
        delegate_ = create_fn_(keys, values, 1, /*err*/ nullptr);
        if (!delegate_) {
            std::cerr << "[Infer] create_fn_ returned nullptr\n";
            dlclose(edgetpu_lib_); edgetpu_lib_ = nullptr;
            return false;
        }
        std::cout << "[Infer] EdgeTPU delegate loaded (device=" << (device?device:"usb:0") << ")\n";
        return true;
    }

};


// =====================================================================
// D. Vision_Postproc (priority 계산 + 정렬)
// =====================================================================

class Vision_Postproc {
public:
    // ID는 Object_Tracker에 의해 할당되므로, 여기서는 priority만 계산
    std::vector<DetectedObject> run(const Vision_RawOut& r, const std::string& scale) {
        // ---- 0) 하이퍼파라미터(필요하면 조절) ----
        constexpr float CONF_TH   = 0.50f;   // 박스 통과 임계값(배경 노이즈 억제)
        constexpr float NMS_TH    = 0.40f;   // IoU 임계값
        constexpr int   TOP_K     = 10;      // 클래스별 최대 유지 수
        constexpr int   MIN_W     = 6;      // 너무 작은 박스 컷 (224 기준)
        constexpr int   MIN_H     = 6;

        // ---- 1) 원시 결과 → 임계값/최소크기 필터 ----
        std::vector<DetectedObject> prelim; prelim.reserve(r.scores.size());
        const int Wc = FRAME_WIDTH/2, Hc = FRAME_HEIGHT/2; // 우선순위(중심근접) 계산용

        for (size_t i = 0; i < r.scores.size(); ++i) {
            float sc = r.scores[i];
            if (sc < CONF_TH) continue;

            // ★ 누락된 부분 복구
            float x1 = r.boxes[i][0];
            float y1 = r.boxes[i][1];
            float x2 = r.boxes[i][2];
            float y2 = r.boxes[i][3];

            float w  = std::max(0.f, x2 - x1);
            float h  = std::max(0.f, y2 - y1);
            float cx = x1 + 0.5f * w;
            float cy = y1 + 0.5f * h;
            if (w < MIN_W || h < MIN_H) continue; //

            DetectedObject o;
            o.id = 0; // 트래커에서 부여
            o.confidence = sc;
            o.obj_class  = std::to_string(r.labels[i]);  // 구조 유지(문자열)

            set_cxywh(o, cx, cy, w, h); // ★ 센터기반으로 채움
            int d2 = (int)((o.bbox[0]-Wc)*(o.bbox[0]-Wc) + (o.bbox[1]-Hc)*(o.bbox[1]-Hc));
            o.priority = d2;
            prelim.push_back(std::move(o));
        }

        if (prelim.empty()) return {};

        // ---- 2) 클래스별로 묶어서 NMS 수행 ----
        std::unordered_map<int, std::vector<int>> idx_by_cls;
        idx_by_cls.reserve(prelim.size());

        for (int i = 0; i < (int)prelim.size(); ++i) {
            int cls_id = 0;
            try { cls_id = std::stoi(prelim[i].obj_class); } catch (...) { cls_id = 0; }
            idx_by_cls[cls_id].push_back(i);
        }

        std::vector<int> keep_all; keep_all.reserve(prelim.size());
        for (auto &kv : idx_by_cls) {
            const auto &indices = kv.second;
            std::vector<cv::Rect> rects; rects.reserve(indices.size());
            std::vector<float> scores;  scores.reserve(indices.size());

            for (int idx : indices) {                    // ← 변수명 idx로
                const auto& o = prelim[idx];
                rects.emplace_back(cxywh_to_tlwh(o));    // ★ CXYWH → TLWH 일시 변환
                scores.push_back(o.confidence);
            }

            std::vector<int> keep;
            cv::dnn::NMSBoxes(rects, scores, 0.0f, NMS_TH, keep, 1.0f, TOP_K);

            for (int j : keep) keep_all.push_back(indices[j]);
        }

        // ---- 3) 결과 재정렬(우선순위 낮은 값이 먼저) + priority 재부여 1..N ----
        std::vector<DetectedObject> out; out.reserve(keep_all.size());
        for (int i : keep_all) out.push_back(prelim[i]);

        std::sort(out.begin(), out.end(),
                [](const DetectedObject& a, const DetectedObject& b){
                    // 1) 중심에서 더 가까운 애 우선
                    if (a.priority != b.priority)
                        return a.priority < b.priority;

                    // 2) 중심거리 같으면 → 더 큰 박스 우선
                    int area_a = a.bbox[2] * a.bbox[3];
                    int area_b = b.bbox[2] * b.bbox[3];
                    if (area_a != area_b)
                        return area_a > area_b;   // ★ 큰 bbox 먼저

                    // 3) 그래도 같으면 → confidence 높은 애 우선
                    return a.confidence > b.confidence;
                });

        for (size_t i = 0; i < out.size(); ++i)
            out[i].priority = (int)i + 1;
        return out;
    }
};

// =====================================================================
// E. Object_Tracker
// =====================================================================

class Object_Tracker {
public:
    Object_Tracker();

    // NMS 이후 dets(CXYWH,int) 입력 → id 부여해서 반환
    std::vector<DetectedObject> track(std::vector<DetectedObject> dets, float dt_ms); //dt_ms(프레임 추가)

    // ---- 공개 유틸(외부에서도 쓰기 쉽게 public static) ----
    static void  cxywh_to_xyxy(const int*, int&, int&, int&, int&);
    static float iou_cxywh(const int*, const int*);

    // ---- 트랙 구조(공개: 디버깅/테스트/툴링 편의) ----
    struct Track {
        int id = 0;
        int bbox[4] = {0,0,0,0};   // CXYWH (정수 캐시)
        float score = 0.f;
        int age = 0;               // 미스 누적 프레임
        int hits = 0;              // 연속 매칭 수
        bool confirmed = false;    // hits >= MIN_HITS
        std::string cls;
        int priority = 0;

        // OpenCV KalmanFilter (x=[cx,cy,w,h,vx,vy], z=[cx,cy,w,h])
        cv::KalmanFilter kf;
        cv::Mat state; // 6x1
        cv::Mat meas;  // 4x1
        bool kf_ready = false;

        inline void init_kf(float cx,float cy,float w,float h);
        inline void predict(float dt_ms);
        inline void correct(float cx,float cy,float w,float h);
        inline void get_pred_bbox_int(int out[4]) const;
    };

private:
    int next_track_id = 1;
    std::unordered_map<int, Track> tracks; // id → Track
};

// ======================= 구현부(헤더-온리) =======================

inline Object_Tracker::Object_Tracker(){
    std::cout << "[Tracker] init (KF)\n";
}

// ---------- Track 구현 ----------
inline void Object_Tracker::Track::init_kf(float cx,float cy,float w,float h){
    kf.init(6, 4, 0, CV_32F);
    state = cv::Mat::zeros(6,1,CV_32F);
    meas  = cv::Mat::zeros(4,1,CV_32F);

    // x = [cx, cy, w, h, vx, vy]
    state.at<float>(0)=cx; state.at<float>(1)=cy;
    state.at<float>(2)=w;  state.at<float>(3)=h;
    state.at<float>(4)=0.f; state.at<float>(5)=0.f;
    kf.statePost = state.clone();

    // H
    kf.measurementMatrix = (cv::Mat_<float>(4,6) <<
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0);

    // 공분산
    cv::setIdentity(kf.errorCovPost, 50.f);

    // Q / R (튜닝 포인트)
    kf.processNoiseCov  = cv::Mat::zeros(6,6,CV_32F);
    kf.processNoiseCov.at<float>(0,0)=1.0f; // pos
    kf.processNoiseCov.at<float>(1,1)=1.0f;
    kf.processNoiseCov.at<float>(2,2)=0.5f; // size
    kf.processNoiseCov.at<float>(3,3)=0.5f;
    kf.processNoiseCov.at<float>(4,4)=5.0f; // vel
    kf.processNoiseCov.at<float>(5,5)=5.0f;

    kf.measurementNoiseCov = cv::Mat::eye(4,4,CV_32F);
    kf.measurementNoiseCov.at<float>(0,0)=3.f;
    kf.measurementNoiseCov.at<float>(1,1)=3.f;
    kf.measurementNoiseCov.at<float>(2,2)=2.f;
    kf.measurementNoiseCov.at<float>(3,3)=2.f;

    kf_ready = true;

    // 캐시
    bbox[0]=(int)std::lround(cx);
    bbox[1]=(int)std::lround(cy);
    bbox[2]=(int)std::lround(w);
    bbox[3]=(int)std::lround(h);
}

inline void Object_Tracker::Track::predict(float dt_ms){
    if (!kf_ready) return;
    const float dt = dt_ms * 1e-3f;
    kf.transitionMatrix = (cv::Mat_<float>(6,6) <<
        1,0,0,0,dt,0,
        0,1,0,0,0,dt,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1);
    cv::Mat pred = kf.predict();
    bbox[0]=(int)std::lround(pred.at<float>(0));
    bbox[1]=(int)std::lround(pred.at<float>(1));
    bbox[2]=(int)std::lround(pred.at<float>(2));
    bbox[3]=(int)std::lround(pred.at<float>(3));
}

inline void Object_Tracker::Track::correct(float cx,float cy,float w,float h){
    if (!kf_ready){ init_kf(cx,cy,w,h); return; }
    meas.at<float>(0)=cx; meas.at<float>(1)=cy;
    meas.at<float>(2)=w;  meas.at<float>(3)=h;
    cv::Mat est = kf.correct(meas);
    bbox[0]=(int)std::lround(est.at<float>(0));
    bbox[1]=(int)std::lround(est.at<float>(1));
    bbox[2]=(int)std::lround(est.at<float>(2));
    bbox[3]=(int)std::lround(est.at<float>(3));
}

inline void Object_Tracker::Track::get_pred_bbox_int(int out[4]) const {
    out[0]=bbox[0]; out[1]=bbox[1]; out[2]=bbox[2]; out[3]=bbox[3];
}

// ---------- 유틸 ----------
inline void Object_Tracker::cxywh_to_xyxy(const int b[4], int& x1,int& y1,int& x2,int& y2){
    x1 = b[0] - b[2]/2;
    y1 = b[1] - b[3]/2;
    x2 = x1 + b[2];
    y2 = y1 + b[3];
}

inline float Object_Tracker::iou_cxywh(const int a[4], const int b[4]){
    int ax1,ay1,ax2,ay2, bx1,by1,bx2,by2;
    cxywh_to_xyxy(a, ax1,ay1,ax2,ay2);
    cxywh_to_xyxy(b, bx1,by1,bx2,by2);
    const int ix0 = std::max(ax1, bx1);
    const int iy0 = std::max(ay1, by1);
    const int ix1 = std::min(ax2, bx2);
    const int iy1 = std::min(ay2, by2);
    const int iw = std::max(0, ix1-ix0);
    const int ih = std::max(0, iy1-iy0);
    const long inter = 1L*iw*ih;
    const long a1 = 1L*(ax2-ax1)*(ay2-ay1);
    const long a2 = 1L*(bx2-bx1)*(by2-by1);
    const long uni = a1 + a2 - inter;
    return (uni>0) ? float(inter)/float(uni) : 0.f;
}

// Greedy 매칭 (IoU 내림차순): pred_tracks ↔ dets
// 헤더 중복정의 방지를 위해 static inline로 내부 연결/인라인화
static inline void greedy_assign_kf(
    const std::vector<int>& track_ids,
    const std::unordered_map<int, Object_Tracker::Track>& tracks,
    const std::vector<int>& det_idx,
    const std::vector<DetectedObject>& dets,
    float iou_th,
    std::vector<std::pair<int,int>>& matches, // (track_id, det_index)
    std::vector<int>& un_trk,                 // track_id
    std::vector<int>& un_det)                 // det_index
{
    struct Cand{ int tid; int di; float iou; };
    std::vector<Cand> cands; cands.reserve(track_ids.size()*det_idx.size());

    int pb[4];
    for (int tid : track_ids){
        auto it = tracks.find(tid);
        if (it == tracks.end()) continue;
        it->second.get_pred_bbox_int(pb);
        for (int di : det_idx){
            float v = Object_Tracker::iou_cxywh(pb, dets[di].bbox);
            if (v >= iou_th) cands.push_back({tid, di, v});
        }
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b){ return a.iou > b.iou; });

    std::unordered_set<int> used_tr, used_de;
    for (auto& c : cands){
        if (used_tr.count(c.tid) || used_de.count(c.di)) continue;
        used_tr.insert(c.tid);
        used_de.insert(c.di);
        matches.emplace_back(c.tid, c.di);
    }
    for (int tid : track_ids) if (!used_tr.count(tid)) un_trk.push_back(tid);
    for (int di  : det_idx)   if (!used_de.count(di))  un_det.push_back(di);
}

inline std::vector<DetectedObject> Object_Tracker::track(std::vector<DetectedObject> dets,float dt_ms) { //dt_ms추가
    // ===== 파라미터 =====
    constexpr float HIGH_TH   = 0.50f;  // 1차 매칭용 점수
    constexpr float LOW_TH    = 0.35f;  // 2차 매칭용 점수
    constexpr float IOU_HIGH  = 0.35f;  // 1차 IoU 게이팅
    constexpr float IOU_LOW   = 0.25f;  // 2차 IoU 게이팅
    constexpr int   MAX_AGE   = 25;     // 미스 허용 프레임
    constexpr int   MIN_HITS  = 2;      // 확정에 필요한 히트 수
    // =====================

    // 0) det 분리
    std::vector<int> H, L; H.reserve(dets.size()); L.reserve(dets.size());
    for (int i=0;i<(int)dets.size();++i){
        if (dets[i].confidence >= HIGH_TH) H.push_back(i);
        else if (dets[i].confidence >= LOW_TH) L.push_back(i);
    }

    // 1) 모든 트랙 예측
    std::vector<int> track_ids; track_ids.reserve(tracks.size());
    for (auto& kv : tracks){
        kv.second.predict(dt_ms);
        track_ids.push_back(kv.first);
    }

    // 2) 1차 매칭 (tracks ↔ H)
    std::vector<std::pair<int,int>> m1;
    std::vector<int> unT1, unH;
    greedy_assign_kf(track_ids, tracks, H, dets, IOU_HIGH, m1, unT1, unH);

    // 3) 2차 매칭 (남은 트랙 ↔ L)
    std::vector<std::pair<int,int>> m2;
    std::vector<int> unT2, unL;
    greedy_assign_kf(unT1, tracks, L, dets, IOU_LOW, m2, unT2, unL);

    // 4) 매칭된 트랙 업데이트(KF correct), 속성 반영
    std::unordered_set<int> det_used; det_used.reserve(dets.size());
    for (auto& pr : m1){
        auto& tr = tracks[pr.first];
        const auto& d = dets[pr.second];
        tr.correct(d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3]);
        tr.score = d.confidence; tr.cls = d.obj_class; tr.priority = d.priority;
        tr.age = 0; tr.hits++; if (!tr.confirmed && tr.hits >= MIN_HITS) tr.confirmed = true;
        det_used.insert(pr.second);
        dets[pr.second].id = tr.id;
    }
    for (auto& pr : m2){
        auto& tr = tracks[pr.first];
        const auto& d = dets[pr.second];
        tr.correct(d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3]);
        tr.score = d.confidence; tr.cls = d.obj_class; tr.priority = d.priority;
        tr.age = 0; tr.hits++; if (!tr.confirmed && tr.hits >= MIN_HITS) tr.confirmed = true;
        det_used.insert(pr.second);
        dets[pr.second].id = tr.id;
    }

    // 5) 새 트랙 생성 (unH만; L로는 생성 X)
    for (int di : unH){
        if (det_used.count(di)) continue;
        const auto& d = dets[di];
        Track t{};
        t.id = next_track_id++;
        t.init_kf((float)d.bbox[0], (float)d.bbox[1], (float)d.bbox[2], (float)d.bbox[3]);
        t.score = d.confidence; t.cls = d.obj_class; t.priority = d.priority;
        t.age = 0; t.hits = 1; t.confirmed = (MIN_HITS<=1);
        tracks[t.id] = std::move(t);
        dets[di].id = tracks[t.id].id;
    }

    // 6) 미스 증가/정리
    std::vector<int> to_erase;
    for (auto& kv : tracks){
        int tid = kv.first;
        bool matched = false;
        for (auto& p : m1) if (p.first==tid) { matched=true; break; }
        if (!matched) for (auto& p : m2) if (p.first==tid) { matched=true; break; }
        if (!matched) kv.second.age++;
        if (kv.second.age > MAX_AGE) to_erase.push_back(tid);
    }
    for (int tid : to_erase) tracks.erase(tid);

    // 7) 출력: 매칭된 det는 id가 채워져 있음. (미확정 포함)
    return dets;
}
// =====================================================================
// F. Target_Manager
// =====================================================================
class Target_Manager {
private:
    int target_track_id = 0;
    std::vector<int> fired_ids;
public:
    // Tracking 모듈이 ID를 부여했으므로, 이 ID를 기반으로 타겟 선택
    DetectedObject getTrackedTarget(const std::vector<DetectedObject>& dets);
    bool isFired(int id);
    void addFiredId(int id);
    void clearFiredIds(); // 발사 기록 초기화 함수 추가
};

// ================== 구현부 (헤더-온리, 원본 로직 그대로) ==================

inline DetectedObject Target_Manager::getTrackedTarget(const std::vector<DetectedObject>& dets) {
    std::vector<DetectedObject> valid;
    for (const auto& d: dets){
        if (d.id!=0 && !isFired(d.id)) valid.push_back(d);
    }
    if (valid.empty()){ target_track_id = 0; return DetectedObject{0}; }

    auto best = *std::min_element(valid.begin(), valid.end(),
                 [](auto& a, auto& b){ return a.priority < b.priority; });

    //target_track_id = best.id;   // 원하면 이 줄도 지워도 됨 (락온 안 쓸 거면)
    return best;
}


/*
inline DetectedObject Target_Manager::getTrackedTarget(const std::vector<DetectedObject>& dets) {
    std::vector<DetectedObject> valid;
    for (const auto& d: dets){
        if (d.id!=0 && !isFired(d.id)) valid.push_back(d);
    }
    if (valid.empty()){ target_track_id = 0; return DetectedObject{0}; }

    if (target_track_id!=0){
        for (const auto& d: valid) if (d.id==target_track_id) return d;
        //std::cout << "[TargetMgr] lost id="<<target_track_id<<", reselect\n";
        target_track_id = 0;
    }
    auto best = *std::min_element(valid.begin(), valid.end(),
                 [](auto& a, auto& b){ return a.priority < b.priority; });
    target_track_id = best.id;
    return best;
}
*/

inline bool Target_Manager::isFired(int id){
    for (int v: fired_ids) if (v==id) return true;
    return false;
}

inline void Target_Manager::addFiredId(int id){
    if (!isFired(id)){ fired_ids.push_back(id); std::cout<<"[TargetMgr] fired "<<id<<"\n"; }
}

inline void Target_Manager::clearFiredIds() { fired_ids.clear(); } // 발사 기록 초기화 함수

// =====================================================================
// G. Transport_PC (삭제됨)
// =====================================================================

// =====================================================================
// H. Transport_Gimbal (삭제됨)
// =====================================================================