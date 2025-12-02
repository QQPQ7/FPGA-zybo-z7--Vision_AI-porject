#pragma once
#include <string>

namespace gpio {

int  export_pin(int id);
int  unexport_pin(int id);
int  set_dir(int id, bool is_out);
int  write(int id, int value);
int  read(int id);  // returns 0/1, or -1 on error

} // namespace gpio
