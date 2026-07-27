#pragma once

namespace sdcard {

bool init();

bool create_dir(const char *full_path);

bool create_logfile(const char *full_path);

void write_log(const char *log_path, const char *log_entry);

int count_files(const char *full_path);

} // namespace sdcard
