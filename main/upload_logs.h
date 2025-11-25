#pragma once
#include <esp_err.h>
void upload_all_logs(const char *server_base_url, const char *device_id);
esp_err_t upload_single_logfile(
    const char *server_base_url,
    const char *device_id,
    const char *filepath,
    const char *filename);