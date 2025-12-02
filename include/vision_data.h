#ifndef VISION_DATA_H
#define VISION_DATA_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <mutex>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream> // for Transport_PC serialization

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/delegates/external/external_delegate.h>

// 추가하려다가 취소
/*
struct SharedDetBuf {
    std::mutex m;
    std::vector<DetectedObject> dets;  // (x,y,w,h), id/priority 포함
};
static SharedDetBuf g_detbuf;
static inline void publish_dets(const std::vector<DetectedObject>& dets){
    std::lock_guard<std::mutex> lk(g_detbuf.m);
    g_detbuf.dets = dets; // copy
}
*/

// ----------------------
// CONSTANTS
// ----------------------
constexpr int FRAME_WIDTH = 640;
constexpr int FRAME_HEIGHT = 480;
constexpr int FRAME_RATE = 30; // 30 FPS

// ----------------------
// DATA STRUCTURES
// ----------------------

// 사용하려다가 취소
/*
// Sensor_Frame: 카메라에서 읽어온 원시 데이터
struct Sensor_Frame {
    int w = 0;
    int h = 0;
    long ts = 0; // Timestamp (milliseconds)
    std::vector<uint8_t> data;
};
*/

// int8_Tensor: 추론 엔진으로 전달되는 전처리된 텐서 (int8_t 요구 모델용)
struct int8_Tensor {
    std::vector<int> shape;
    std::vector<int8_t> data; // uint8_t -> int8_t로 변경
};

// Vision_RawOut: 추론 엔진의 원시 출력
struct Vision_RawOut {
    std::vector<std::vector<float>> boxes; // [N][4] - [xmin, ymin, xmax, ymax]
    std::vector<float> scores;
    std::vector<int> labels;
};

// DetectedObject: 후처리된 객체 정보 (메타데이터로 전송될 핵심 정보)
struct DetectedObject {
    int id = 0;
    std::string obj_class;
    float confidence = 0.0f;
    int bbox[4] = {0, 0, 0, 0}; // [cx, cy, w, h]
    int priority = 9999; // 1: Highest Priority (closest to center), Larger number: Lower Priority
};

#endif // VISION_DATA_H

