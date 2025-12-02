#include "hcsr04_sensor.hpp"
#include <cstdio>
#include <unistd.h>

int main() {
    Hcsr04Sensor s(/*trig*/516, /*echo*/518);
    if (s.init() != 0) { std::puts("init failed"); return 1; }

    while(1) {
        double cm = s.measure_once();
        if (cm < 0) std::printf("err=%.0f\n", cm);
        else        std::printf("%.2f cm\n", cm);
        usleep(100000);
    }
    s.deinit();
}