#pragma once
#include <nlohmann/json.hpp>
#include <array>
#include <vector>
#include <string>
#include <optional>
#include <atomic>


// 공통 컨테이너 (소문자 키)
struct Message {
    std::string type;
    std::string timestamp;
    nlohmann::json data;
};

// ---------- Zynq -> PC ----------
struct StatusData {  
    float yaw{0.0};
    float pitch{0.0};
};

struct BBoxObject {
    int id{0};
    std::string class_name; // JSON 키는 "class"
    double confidence{0.0};
    std::array<int,4> bbox{0,0,0,0};    // [x1,y1,x2,y2]
    int priority{0};
};
struct BBoxData {
    int frame_id{0};
    std::vector<BBoxObject> objects;
};

struct FireReadyData {
    int  target_id{0};
    bool ready{false};
    double distance{0.0};
};

struct FireDoneData {
    int  target_id{0};
};

// ---------- PC -> Zynq ----------
struct GimbalControl   { float yaw{0.0}, pitch{0.0}; };
struct FireMode        { int mode{0}; };          // 0:수동 1:자동
struct OperationMode   { int mode{0}; };          // 0:수동 1:자동
struct FireCommand     { bool trigger{false}; };
struct LaserControl    { int  state{0}; };        // 0:Off 1:On
struct TrackTarget     { int  target_id{0}; };
struct TrackRelease    { /* empty */ };
struct FireResult      { bool success{false}; };
struct RobotMoving     { int  moving{0}; };       // 0:정지 1:전진
struct SoftReset   { bool reset{false}; };
// --------------- JSON 변환 ---------------
inline void to_json(nlohmann::json& j, const Message& m){
    j = nlohmann::json{{"type", m.type}, {"timestamp", m.timestamp}, {"data", m.data}};
}
inline void from_json(const nlohmann::json& j, Message& m){
    j.at("type").get_to(m.type);
    j.at("timestamp").get_to(m.timestamp);
    if (j.contains("data")) m.data = j.at("data");
    else m.data = nlohmann::json::object();
}

// Status
inline void to_json(nlohmann::json& j, const StatusData& p){
    j = {{"yaw", p.yaw}, {"pitch", p.pitch}};
}
inline void from_json(const nlohmann::json& j, StatusData& p){
    if (j.contains("yaw"))   p.yaw   = j.at("yaw").get<float>();
    if (j.contains("pitch"))   p.pitch   = j.at("pitch").get<float>();
}

// BBox
inline void to_json(nlohmann::json& j, const BBoxObject& o){
    j = {{"id", o.id}, {"class", o.class_name}, {"confidence", o.confidence}, {"bbox", o.bbox}, {"priority", o.priority}};
}
inline void from_json(const nlohmann::json& j, BBoxObject& o){
    if (j.contains("id"))         o.id = j.at("id").get<int>();
    if (j.contains("class"))      o.class_name = j.at("class").get<std::string>();
    if (j.contains("confidence")) o.confidence = j.at("confidence").get<double>();
    if (j.contains("bbox"))       o.bbox = j.at("bbox").get<std::array<int,4>>();
}
inline void to_json(nlohmann::json& j, const BBoxData& b){
    j = {{"frame_id", b.frame_id}, {"objects", b.objects}};
}
inline void from_json(const nlohmann::json& j, BBoxData& b){
    if (j.contains("frame_id")) b.frame_id = j.at("frame_id").get<int>();
    if (j.contains("objects"))  b.objects  = j.at("objects").get<std::vector<BBoxObject>>();
}

// FireReady
inline void to_json(nlohmann::json& j, const FireReadyData& f){
    j = {{"target_id", f.target_id}, {"ready", f.ready}, {"distance", f.distance}};
}
inline void from_json(const nlohmann::json& j, FireReadyData& f){
    if (j.contains("target_id")) f.target_id = j.at("target_id").get<int>();
    if (j.contains("ready"))     f.ready     = j.at("ready").get<bool>();
    if (j.contains("distance"))  f.distance  = j.at("distance").get<double>();
}

// FireDone
inline void to_json(nlohmann::json& j, const FireDoneData& f){
    j = {{"target_id", f.target_id}};
}
inline void from_json(const nlohmann::json& j, FireDoneData& f){
    if (j.contains("target_id")) f.target_id = j.at("target_id").get<int>();
}

// 다운링크(PC->Zynq)
inline void to_json(nlohmann::json& j, const GimbalControl& v){ j = {{"yaw", v.yaw}, {"pitch", v.pitch}}; }
inline void from_json(const nlohmann::json& j, GimbalControl& v){
    if (j.contains("yaw"))   v.yaw   = j.at("yaw").get<float>();
    if (j.contains("pitch")) v.pitch = j.at("pitch").get<float>();
}
inline void to_json(nlohmann::json& j, const FireMode& v){ j = {{"mode", v.mode}}; }
inline void from_json(const nlohmann::json& j, FireMode& v){ if (j.contains("mode")) v.mode = j.at("mode").get<int>(); }
inline void to_json(nlohmann::json& j, const OperationMode& v){ j = {{"mode", v.mode}}; }
inline void from_json(const nlohmann::json& j, OperationMode& v){ if (j.contains("mode")) v.mode = j.at("mode").get<int>(); }
inline void to_json(nlohmann::json& j, const FireCommand& v){ j = {{"trigger", v.trigger}}; }
inline void from_json(const nlohmann::json& j, FireCommand& v){ if (j.contains("trigger")) v.trigger = j.at("trigger").get<bool>(); }
inline void to_json(nlohmann::json& j, const LaserControl& v){ j = {{"state", v.state}}; }
inline void from_json(const nlohmann::json& j, LaserControl& v){ if (j.contains("state")) v.state = j.at("state").get<int>(); }
inline void to_json(nlohmann::json& j, const TrackTarget& v){ j = {{"target_id", v.target_id}}; }
inline void from_json(const nlohmann::json& j, TrackTarget& v){ if (j.contains("target_id")) v.target_id = j.at("target_id").get<int>(); }
inline void to_json(nlohmann::json& j, const TrackRelease&){ j = nlohmann::json::object(); }
inline void from_json(const nlohmann::json&, TrackRelease&){ }
inline void to_json(nlohmann::json& j, const FireResult& v){ j = {{"success", v.success}}; }
inline void from_json(const nlohmann::json& j, FireResult& v){ if (j.contains("success")) v.success = j.at("success").get<bool>(); }
inline void to_json(nlohmann::json& j, const RobotMoving& v){ j = {{"moving", v.moving}}; }
inline void from_json(const nlohmann::json& j, RobotMoving& v){ if (j.contains("moving")) v.moving = j.at("moving").get<int>(); }
inline void to_json(nlohmann::json& j, const SoftReset v){ j = {{"reset", v.reset}}; }
inline void from_json(const nlohmann::json& j, SoftReset& v){ if (j.contains("reset")) v.reset = j.at("reset").get<bool>(); }