#include "urban_gimbalstable.hpp"
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <clocale>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

// ===== 전역 포인터 (SIGINT 처리용) =====
static GimbalStabilizer* g_stabilizer = nullptr;

// 터미널 raw 모드 설정용 RAII
struct TermRaw {
    termios old{};
    bool ok{false};

    TermRaw() {
        if (tcgetattr(STDIN_FILENO, &old) == -1) return;
        termios raw = old;
        raw.c_lflag &= ~(ICANON | ECHO);   // 비캐논, 에코 끄기
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) return;

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1) {
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK); // 논블로킹
        }
        ok = true;
    }

    ~TermRaw() {
        if (!ok) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags != -1) {
            fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        }
    }
};

void sigint_handler(int) {
    if (g_stabilizer) {
        g_stabilizer->request_stop();
        g_stabilizer->deinit();
        printf("\n[INFO] 종료 신호 수신\n");
    }
    std::_Exit(0);
}

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    setlocale(LC_ALL, "");

    GimbalStabilizer stabilizer;
    g_stabilizer = &stabilizer;

    std::signal(SIGINT, sigint_handler);

    if (!stabilizer.init()) return 1;

    // IMU 스레드 시작
    std::thread imu_thread(&IMUReader::run_loop, &stabilizer.imu_, 200.0);

    // === 초기 목표각 설정 ===
    float tgt_yaw_deg   = YAW_OFFSET_DEG;  // Yaw는 오프셋만
    float tgt_pitch_deg = 0.0f;            // Pitch 0도에서 시작
    stabilizer.set_target_angles(tgt_yaw_deg, tgt_pitch_deg);

    // 제어 루프 스레드 시작
    std::thread stabilize_thread(&GimbalStabilizer::stabilize_loop, &stabilizer);

    // 키 입력을 위해 터미널을 raw + non-blocking 모드로 전환
    TermRaw term;

    const float YAW_STEP_DEG   = 1.0f;   // yaw 한 번에 움직일 각도
    const float PITCH_STEP_DEG = 1.0f;   // pitch 한 번에 움직일 각도

    printf("[CTRL] 방향키로 짐벌 제어 (←/→: Yaw, ↑/↓: Pitch, q: 종료)\n");

    bool running = true;
    while (running) {
        char buf[3];
        ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));

        if (n > 0) {
            // q / Q 로 종료
            if (buf[0] == 'q' || buf[0] == 'Q') {
                printf("\n[CTRL] 'q' 입력, 종료합니다.\n");
                running = false;
            }
            // 방향키: ESC [ A/B/C/D
            else if (buf[0] == 27) { // ESC
                char seq[2];
                ssize_t n2 = read(STDIN_FILENO, seq, 2);
                if (n2 == 2 && seq[0] == '[') {
                    bool updated = false;
                    switch (seq[1]) {
                        case 'A': // ↑ : pitch 증가
                            tgt_pitch_deg += PITCH_STEP_DEG;
                            updated = true;
                            break;
                        case 'B': // ↓ : pitch 감소
                            tgt_pitch_deg -= PITCH_STEP_DEG;
                            updated = true;
                            break;
                        case 'C': // → : yaw 증가
                            tgt_yaw_deg += YAW_STEP_DEG;
                            updated = true;
                            break;
                        case 'D': // ← : yaw 감소
                            tgt_yaw_deg -= YAW_STEP_DEG;
                            updated = true;
                            break;
                        default:
                            break;
                    }

                    if (updated) {
                        // 각도 클램프 (헤더에 정의된 범위 사용)
                        if (tgt_yaw_deg < -90) tgt_yaw_deg = -90;
                        if (tgt_yaw_deg > 90) tgt_yaw_deg = 90;
                        if (tgt_pitch_deg < -90) tgt_pitch_deg = -90;
                        if (tgt_pitch_deg > 90) tgt_pitch_deg = 90;

                        stabilizer.set_target_angles(tgt_yaw_deg, tgt_pitch_deg);
                        printf("\r[CTRL] Yaw = %6.2f deg, Pitch = %6.2f deg   ",
                               tgt_yaw_deg, tgt_pitch_deg);
                        fflush(stdout);
                    }
                }
            }
            // 그 외 키는 무시
        }

        // 너무 바쁘지 않게 약간 쉼
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // === 정리 ===
    stabilizer.request_stop();

    if (stabilize_thread.joinable())
        stabilize_thread.join();
    if (imu_thread.joinable())
        imu_thread.join();

    stabilizer.deinit();
    g_stabilizer = nullptr;

    printf("\n[MAIN] 프로그램 정상 종료\n");
    return 0;
}
