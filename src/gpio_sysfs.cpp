#include "gpio_sysfs.hpp"
#include <cstdio>
#include <cstring>

namespace {

int write_str(const char* path, const char* s) {
    if (FILE* f = ::fopen(path, "w")) {
        int r = ::fprintf(f, "%s", s);
        ::fclose(f);
        return r < 0 ? -1 : 0;
    }
    return -1;
}

} // anonymous

namespace gpio {

int export_pin(int id) {
    char b[32];
    ::snprintf(b, sizeof b, "%d", id);
    return write_str("/sys/class/gpio/export", b);
}

int unexport_pin(int id) {
    char b[32];
    ::snprintf(b, sizeof b, "%d", id);
    return write_str("/sys/class/gpio/unexport", b);
}

int set_dir(int id, bool is_out) {
    char p[128];
    ::snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/direction", id);
    return write_str(p, is_out ? "out" : "in");
}

int write(int id, int value) {
    char p[128], v[4];
    ::snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/value", id);
    ::snprintf(v, sizeof v, "%d", value ? 1 : 0);
    return write_str(p, v);
}

int read(int id) {
    char p[128];
    ::snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/value", id);
    if (FILE* f = ::fopen(p, "r")) {
        int v = -1;
        if (::fscanf(f, "%d", &v) != 1) v = -1;
        ::fclose(f);
        return v;
    }
    return -1;
}

} // namespace gpio
