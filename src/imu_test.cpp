#include <csignal>
#include <thread>
#include <chrono>
#include <cstdio>
#include <atomic>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <cmath>        // RAD2DEG

#include "imu.hpp"
#include "imu_data.h"

static std::atomic<bool> g_running{true};
static std::mutex g_print_mtx;

static void on_sigint(int) { g_running = false; }

int main() {
    // Ctrl+C 종료 플래그
    std::signal(SIGINT, on_sigint);

    IMUReader imu_head("Head");
    imu_head.setDevice("/dev/i2c-0");

    if (!imu_head.init(true)) {
        std::fprintf(stderr, "[MAIN] IMU init failed\n");
        return 1;
    }

    // 출력 버퍼링을 줄여 곧바로 화면 반영
    setvbuf(stdout, nullptr, _IONBF, 0);

    // ✅ CSV 파일 열기
    std::ofstream csv("imu_pitch_only.csv");
    if (!csv.is_open()) {
        std::fprintf(stderr, "[MAIN] failed to open imu_pitch_only.csv\n");
        return 1;
    }

    // ✅ CSV 헤더: pitch_filt 하나만
    csv << "pitch_filt\n";
    csv << std::fixed;

    // 타임스탬프용 (필요 없으면 통째로 빼도 됨)
    double prev_ts = 0.0;
    bool first_sample = true;

    // IMU 읽기 스레드 (센서 폴링 루프)
    std::thread th([&](){
        imu_head.run_loop(200.0); // 200 Hz 루프
    });

    // 커서 숨김
    std::fputs("\x1b[?25l", stdout);

    std::puts("[MAIN] ICM-20948 reading... (Ctrl+C to quit)");

    // 한 줄 갱신 루프
    while (g_running.load()) {
        IMUData d{};
        double ts = 0.0;
        uint64_t seq = 0;

        if (imu_head.latest(d, ts, &seq)) {
            // mAngles: [0]=Roll, [1]=Pitch, [2]=Yaw  (deg, 필터 결과)
            double pitch_filt = d.mAngles[1];

            // 필요하다면 콘솔에도 찍기
            std::cout << pitch_filt << "\n";

            // ✅ CSV에 pitch_filt만 기록
            csv << std::setprecision(6)
                << pitch_filt << "\n";
        }

        // 센서 폴링과 경쟁하지 않도록 아주 짧은 휴식
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (th.joinable()) th.join();

    // 파일 정리
    csv.close();

    // 커서 복원 + 줄 바꿈
    std::fputs("\x1b[?25h\n", stdout);
    std::puts("[MAIN] bye.");
    return 0;
}
