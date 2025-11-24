#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"

#include "main.h" // voor mqtt_publish_status
#include "ota.h"

static const char *TAG = "OTA";

static esp_err_t do_http_ota(const char *url)
{
    esp_http_client_config_t http_cfg = {
        .url = url,
        .timeout_ms = 15000,
        // LET OP: geen transport_type, geen TLS-config → plain HTTP
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Failed to init http client");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "HTTP status = %d, content_length = %d", status_code, content_length);

    if (status_code != 200)
    {
        ESP_LOGE(TAG, "Non-200 status code");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);
    if (!update_partition)
    {
        ESP_LOGE(TAG, "No OTA partition available");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%" PRIx32,
             update_partition->subtype, update_partition->address);

    esp_ota_handle_t ota_handle = 0;
    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return err;
    }

    uint8_t buf[4096];
    int total_written = 0;

    while (1)
    {
        int read = esp_http_client_read(client, (char *)buf, sizeof(buf));
        if (read < 0)
        {
            ESP_LOGE(TAG, "HTTP read error");
            err = ESP_FAIL;
            break;
        }
        else if (read == 0)
        {
            // einde
            break;
        }

        err = esp_ota_write(ota_handle, buf, read);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            break;
        }
        total_written += read;
    }

    ESP_LOGI(TAG, "OTA written %d bytes", total_written);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK)
    {
        esp_ota_end(ota_handle);
        return err;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "OTA update applied, will boot from new partition");
    return ESP_OK;
}

static void ota_task(void *pvParameter)
{
    char *url = (char *)pvParameter;

    ESP_LOGI(TAG, "Starting OTA from URL: %s", url);
    mqtt_publish_status("updating_firmware");

    esp_err_t ret = do_http_ota(url);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA successful, rebooting");
        mqtt_publish_status("update_ok_rebooting");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(ret));
        mqtt_publish_status("update_failed");
    }

    free(url);
    vTaskDelete(NULL);
}

void ota_start_from_url(const char *url)
{
    if (!url || strlen(url) == 0)
    {
        ESP_LOGW(TAG, "Empty OTA URL");
        mqtt_publish_status("update_invalid_url");
        return;
    }

    char *copy = strdup(url);
    if (!copy)
    {
        mqtt_publish_status("update_no_mem");
        return;
    }

    if (xTaskCreate(ota_task, "ota_task", 8192, copy, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create OTA task");
        free(copy);
        mqtt_publish_status("update_task_failed");
    }
}
