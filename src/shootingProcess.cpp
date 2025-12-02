#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <optional>
#include <filesystem>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

// ====== 보드별로 맞춰야 하는 부분 ======
static constexpr int JE1_GPIO = 512;   // ← JE1의 sysfs GPIO 번호로 바꿔주세요
static constexpr bool ACTIVE_LOW = false; // 배선상 Low가 켜짐이면 true
// =====================================

static bool path_exists(const std::string& p) {
    struct stat st{};
    return ::stat(p.c_str(), &st) == 0;
}

static int write_text(const std::string& p, const std::string& s) {
    int fd = ::open(p.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) {
        std::perror(("open " + p).c_str());
        return -1;
    }
    ssize_t n = ::write(fd, s.c_str(), s.size());
    if (n < 0 || (size_t)n != s.size()) {
        std::perror(("write " + p).c_str());
        ::close(fd);
        return -1;
    }
    ::close(fd);
    return 0;
}

class GpioOut {
public:
    explicit GpioOut(int gpio, bool active_low=false)
        : gpio_(gpio), active_low_(active_low) {}

    bool init() {
        // 1) export
        const std::string gp = "/sys/class/gpio/gpio" + std::to_string(gpio_);
        if (!path_exists(gp)) {
            if (write_text("/sys/class/gpio/export", std::to_string(gpio_)) < 0) {
                std::fprintf(stderr, "export %d failed\n", gpio_);
                return false;
            }
        }
        // sysfs 경로가 생길 때까지 잠깐 대기
        for (int i = 0; i < 50 && !path_exists(gp); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 2) direction=out
        if (write_text(gp + "/direction", "out") < 0) return false;

        // 3) active_low (없을 수도 있으니 실패해도 치명적 아님)
        (void)write_text(gp + "/active_low", active_low_ ? "1" : "0");

        // 4) value 파일 open 고정(성능/안정성)
        value_path_ = gp + "/value";
        value_fd_ = ::open(value_path_.c_str(), O_WRONLY | O_CLOEXEC);
        if (value_fd_ < 0) {
            std::perror(("open " + value_path_).c_str());
            return false;
        }
        return true;
    }

    void on()  { set(true);  }
    void off() { set(false); }

    void blink(int count, int ms) {
        for (int i = 0; i < count; ++i) {
            on();  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            off(); std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
    }

    void set(bool high) {
        // active_low는 커널에 설정했으므로 논리값 그대로 1/0을 씀
        if (value_fd_ < 0) return;
        const char* s = high ? "1" : "0";
        ssize_t n = ::pwrite(value_fd_, s, 1, 0);
        if (n != 1) {
            // fallback: 재오픈
            ::close(value_fd_);
            value_fd_ = ::open(value_path_.c_str(), O_WRONLY | O_CLOEXEC);
            if (value_fd_ >= 0) {
                (void)::pwrite(value_fd_, s, 1, 0);
            } else {
                std::perror(("reopen " + value_path_).c_str());
            }
        }
    }

    ~GpioOut() {
        if (value_fd_ >= 0) ::close(value_fd_);
        // 보통은 unexport하지 않고 남겨두는 게 여러 프로세스 협업에 안전
        // 필요하면 아래 주석 해제
        // write_text("/sys/class/gpio/unexport", std::to_string(gpio_));
    }

private:
    int gpio_;
    bool active_low_;
    std::string value_path_;
    int value_fd_{-1};
};

struct Ina219Sample {
    long long ts_ms{0};   // epoch ms
    int mV{-1};           // millivolts
    int mA{-1};           // milliamps
    long long uW{-1};     // microwatts
};

class Ina219Hwmon {
public:
    // hint 예: "1-0040" (i2c-1 @ 0x40) 또는 빈 문자열(자동 탐색)
    explicit Ina219Hwmon(std::string device_hint = "") {
        base_ = find_hwmon_base(device_hint);
        ok_ = !base_.empty();
        if (!ok_) {
            std::cerr << "[INA219] hwmon path not found\n";
        }
    }

    bool ok() const { return ok_; }
    const std::string& base() const { return base_; }

    // 단일 샘플 즉시 읽기
    std::optional<Ina219Sample> read_once() const {
        if (!ok_) return std::nullopt;
        Ina219Sample s;
        s.ts_ms = now_ms();

        // 전압(mV), 전류(mA), 전력(uW)
        auto in1 = read_int_file(base_ + "/in1_input");     // may be absent depending on driver
        auto cur = read_int_file(base_ + "/curr1_input");   // required by ina219 hwmon
        auto pow = read_int_file(base_ + "/power1_input");  // optional

        if (in1) s.mV = *in1;
        if (cur) s.mA = *cur;
        if (pow) s.uW = *pow;

        // power1_input이 없으면 mV * mA로 근사 (uW 단위)
        if (!pow && in1 && cur) {
            s.uW = static_cast<long long>(s.mV) * static_cast<long long>(s.mA);
        }
        return s;
    }

    // 백그라운드 폴링 시작/중지
    void start(int period_ms = 200) {
        if (!ok_) return;
        stop();
        run_.store(true);
        th_ = std::thread([this, period_ms]{
            while (run_.load()) {
                auto s = read_once();
                if (s) {
                    std::lock_guard<std::mutex> lk(mx_);
                    latest_ = *s;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            }
        });
    }
    void stop() {
        run_.store(false);
        if (th_.joinable()) th_.join();
    }
    Ina219Sample latest() const {
        std::lock_guard<std::mutex> lk(mx_);
        return latest_;
    }
    ~Ina219Hwmon(){ stop(); }

private:
    static long long now_ms(){
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }
    static std::optional<int> read_int_file(const std::string& p){
        std::ifstream ifs(p);
        if (!ifs) return std::nullopt;
        long long v = 0;
        ifs >> v;
        if (!ifs.good() && !ifs.eof()) return std::nullopt;
        return static_cast<int>(v);
    }
    static std::string read_first_line(const fs::path& p){
        std::ifstream ifs(p);
        std::string line;
        std::getline(ifs, line);
        return line;
    }

    // hwmon 경로 탐색:
    //  1) device_hint가 "bus-addr"(예: "1-0040")면 /sys/bus/i2c/devices/<hint>/hwmon/hwmon*/ 우선
    //  2) 아니면 /sys/class/hwmon/hwmon*/name에서 "ina2xx"/"ina219" 찾기
    static std::string find_hwmon_base(const std::string& hint){
        // 1) 힌트 우선
        if (!hint.empty()) {
            fs::path try1 = fs::path("/sys/bus/i2c/devices") / hint / "hwmon";
            if (fs::exists(try1) && fs::is_directory(try1)) {
                for (auto& d : fs::directory_iterator(try1)) {
                    if (fs::is_directory(d)) {
                        fs::path name = d.path() / "name";
                        if (fs::exists(name)) {
                            auto n = read_first_line(name);
                            if (n.rfind("ina", 0) == 0) {
                                return d.path().string();
                            }
                        } else {
                            // 어떤 보드는 name이 상위에만 있을 수도 있어 바로 채택
                            return d.path().string();
                        }
                    }
                }
            }
        }
        // 2) 클래스 탐색
        fs::path root = "/sys/class/hwmon";
        if (fs::exists(root) && fs::is_directory(root)) {
            for (auto& d : fs::directory_iterator(root)) {
                fs::path name = d.path() / "name";
                if (!fs::exists(name)) continue;
                auto n = read_first_line(name);
                if (n == "ina2xx" || n == "ina219") {
                    return d.path().string();
                }
            }
        }
        return {};
    }

    std::string base_;
    bool ok_{false};
    std::atomic<bool> run_{false};
    mutable std::mutex mx_;
    Ina219Sample latest_{};
    std::thread th_;
};

int main(int argc, char** argv){
    GpioOut JE1(JE1_GPIO, true);
    if (!JE1.init()) {
        std::fprintf(stderr, "JE1 GPIO init failed (gpio=%d)\n", JE1_GPIO);
        return 1;
    }
    // (선택) 인자로 i2c-버스/주소 힌트 전달: 예) ./ina219_read 1-0040
    std::string hint = (argc >= 2) ? argv[1] : "";
    Ina219Hwmon ina(hint);
    if (!ina.ok()) return 1;
    
    int preCurrent = 0;
    while(true){
        bool shootingComplete = false;
        JE1.on();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while(shootingComplete != true){
            auto s = ina.read_once();
            if(s->mA - preCurrent <= -15){
                std::cout << "shooting complete" << "\n";
                shootingComplete = true;
            } 
            preCurrent = s->mA;
            std::cout << s->mA << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        JE1.off();
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

    return 0;
}
