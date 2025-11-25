#include "esp_littlefs.h"
#include "esp_log.h"

static const char *TAG = "logfs";

void init_log_fs(void)
{
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/logs",      // pad waaronder de FS zichtbaar is
        .partition_label = "logs", // moet overeenkomen met Name in partitions.csv
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount LittleFS (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info("logs", &total, &used);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "LittleFS total=%u, used=%u", (unsigned)total, (unsigned)used);
    }
    else
    {
        ESP_LOGW(TAG, "Failed to get LittleFS info (%s)", esp_err_to_name(ret));
    }
}

void create_test_log(void)
{
    ESP_LOGI("log_upload", "create_test_log: trying to create /logs/testlog.bin");
    FILE *f = fopen("/logs/testlog.bin", "wb");
    if (!f)
    {
        ESP_LOGE("log_upload", "create_test_log: fopen failed");
        return;
    }

    const char buf[] = "hello from esp";
    size_t written = fwrite(buf, 1, sizeof(buf), f);
    fclose(f);

    ESP_LOGI("log_upload", "create_test_log: wrote %u bytes", (unsigned)written);
}
