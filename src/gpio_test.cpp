#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>


// sudo ./build/gpio_output 512 on

// # 끄기
// sudo ./build/gpio_output 512 off

// # 깜빡이기: 5회, 200ms
// sudo ./build/gpio_output 512 blink 5 200

// 간단한 파일 쓰기 유틸
static int write_file(const std::string& path, const std::string& s) {
    std::ofstream f(path);
    if (!f) {
        std::perror(("open " + path).c_str());
        return -1;
    }
    f << s;
    if (!f.good()) {
        std::fprintf(stderr, "write failed: %s\n", path.c_str());
        return -1;
    }
    return 0;
}

static bool path_exists(const std::string& path) {
    struct stat st{};
    return ::stat(path.c_str(), &st) == 0;
}

// sysfs GPIO helpers
static int gpio_export(int pin) {
    const std::string gp = "/sys/class/gpio/gpio" + std::to_string(pin);
    if (path_exists(gp)) return 0; // already exported
    return write_file("/sys/class/gpio/export", std::to_string(pin));
}

static int gpio_unexport(int pin) {
    const std::string gp = "/sys/class/gpio/gpio" + std::to_string(pin);
    if (!path_exists(gp)) return 0;
    return write_file("/sys/class/gpio/unexport", std::to_string(pin));
}

static int gpio_set_dir(int pin, bool output) {
    const std::string p = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    return write_file(p, output ? "out" : "in");
}

static int gpio_set_active_low(int pin, bool active_low) {
    const std::string p = "/sys/class/gpio/gpio" + std::to_string(pin) + "/active_low";
    if (!path_exists(p)) return 0; // 커널 옵션에 따라 없을 수 있음 → 무시
    return write_file(p, active_low ? "1" : "0");
}

static int gpio_write(int pin, int v) {
    const std::string p = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    return write_file(p, v ? "1" : "0");
}

// 사용법 출력
static void usage(const char* prog) {
    std::fprintf(stderr,
        "Usage:\n"
        "  %s <pin> on               # 지정 GPIO High\n"
        "  %s <pin> off              # 지정 GPIO Low\n"
        "  %s <pin> blink <count> <ms>  # <count>회, 각 펄스 <ms>ms\n"
        "Options:\n"
        "  환경변수 ACTIVE_LOW=1   # active_low=1로 설정(배선이 반대일 때)\n"
        "Examples:\n"
        "  %s 512 on\n"
        "  ACTIVE_LOW=1 %s 512 on\n"
        "  %s 512 blink 5 200\n",
        prog, prog, prog, prog, prog, prog);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }

    int pin = std::atoi(argv[1]);
    std::string cmd = argv[2];

    // 1) export & 설정
    if (gpio_export(pin) < 0) return 1;
    // sysfs 경로가 만들어질 때까지 잠깐 대기 (환경에 따라 필요)
    for (int i=0; i<10 && !path_exists("/sys/class/gpio/gpio"+std::to_string(pin)); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (gpio_set_dir(pin, true) < 0) return 1;

    // active_low 옵션 (배선에 따라 LED가 반대로 동작하면 ACTIVE_LOW=1)
    bool active_low = false;
    if (const char* al = std::getenv("ACTIVE_LOW")) {
        if (std::strcmp(al, "1") == 0 || std::strcmp(al, "true") == 0) active_low = true;
    }
    (void)gpio_set_active_low(pin, active_low);

    // 2) 명령 실행
    if (cmd == "on") {
        if (gpio_write(pin, 1) < 0) return 1;
        std::printf("[OK] gpio%d -> ON%s\n", pin, active_low ? " (active_low)" : "");
        return 0;
    }
    else if (cmd == "off") {
        if (gpio_write(pin, 0) < 0) return 1;
        std::printf("[OK] gpio%d -> OFF%s\n", pin, active_low ? " (active_low)" : "");
        return 0;
    }
    else if (cmd == "blink") {
        if (argc < 5) {
            usage(argv[0]);
            return 1;
        }
        int count = std::atoi(argv[3]);
        int ms = std::atoi(argv[4]);
        if (count <= 0 || ms <= 0) {
            std::fprintf(stderr, "blink 인자는 양수여야 합니다.\n");
            return 1;
        }
        for (int i=0; i<count; ++i) {
            if (gpio_write(pin, 1) < 0) return 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            if (gpio_write(pin, 0) < 0) return 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
        std::printf("[OK] gpio%d blink %d x %dms%s\n", pin, count, ms, active_low ? " (active_low)" : "");
        return 0;
    }

    usage(argv[0]);
    return 1;
}
