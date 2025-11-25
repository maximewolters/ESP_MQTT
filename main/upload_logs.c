#include <dirent.h>
#include <sys/stat.h>
#include <stdio.h>
#include "esp_http_client.h"
#include "esp_log.h"
#include "upload_logs.h"

static const char *TAG = "log_upload";

/**
 * Upload één logfile naar de HTTP server.
 *
 * server_base_url: bv. "http://192.168.1.100:8080"
 * device_id:       bv. "esp-master"
 * filepath:        bv. "/logs/testlog.bin"
 * filename:        bv. "testlog.bin"
 */
esp_err_t upload_single_logfile(
    const char *server_base_url,
    const char *device_id,
    const char *filepath,
    const char *filename)
{
    struct stat st;
    if (stat(filepath, &st) != 0)
    {
        ESP_LOGE(TAG, "stat(%s) failed", filepath);
        return ESP_FAIL;
    }
    size_t file_size = st.st_size;

    /* 1) Volledige URL bouwen: base + pad + query */
    char url[256];
    int n = snprintf(
        url,
        sizeof(url),
        "%s/logs/upload?device_id=%s",
        server_base_url,
        device_id);
    if (n < 0 || n >= (int)sizeof(url))
    {
        ESP_LOGE(TAG, "URL too long, truncated");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Uploading '%s' to '%s' (%u bytes)",
             filepath, url, (unsigned)file_size);

    /* 2) HTTP client configureren met de volledige URL */
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
    {
        ESP_LOGE(TAG, "esp_http_client_init failed");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
    esp_http_client_set_header(client, "X-Log-Filename", filename);

    esp_err_t err = esp_http_client_open(client, file_size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_http_client_open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    FILE *f = fopen(filepath, "rb");
    if (!f)
    {
        ESP_LOGE(TAG, "fopen(%s) failed", filepath);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    uint8_t buf[1024];
    size_t nread;
    while ((nread = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        int w = esp_http_client_write(client, (const char *)buf, nread);
        if (w < 0)
        {
            ESP_LOGE(TAG, "esp_http_client_write failed");
            fclose(f);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
    }

    fclose(f);
    esp_http_client_close(client);

    int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP status for '%s' = %d", filepath, status);

    esp_http_client_cleanup(client);
    return (status == 200) ? ESP_OK : ESP_FAIL;
}

/**
 * Loop over alle files in /logs en upload ze één voor één.
 * Bij succes wordt het bestand na upload verwijderd.
 */
void upload_all_logs(const char *server_base_url, const char *device_id)
{
    ESP_LOGI(TAG, "upload_all_logs: base='%s', device_id='%s'",
             server_base_url, device_id);

    DIR *dir = opendir("/logs");
    if (!dir)
    {
        ESP_LOGW(TAG, "No /logs directory or not mounted");
        return;
    }

    struct dirent *ent;
    int count = 0;
    while ((ent = readdir(dir)) != NULL)
    {
        // sla "." en ".." of verborgen entries over
        if (ent->d_name[0] == '.')
            continue;

        char path[300];
        int n = snprintf(path, sizeof(path), "/logs/%s", ent->d_name);
        if (n < 0 || n >= (int)sizeof(path))
        {
            ESP_LOGE(TAG, "Path truncated for '%s'", ent->d_name);
            continue;
        }

        ESP_LOGI(TAG, "Found log file: %s", path);
        count++;

        if (upload_single_logfile(server_base_url, device_id, path, ent->d_name) == ESP_OK)
        {
            if (remove(path) == 0)
            {
                ESP_LOGI(TAG, "Uploaded & removed %s", path);
            }
            else
            {
                ESP_LOGW(TAG, "Uploaded but failed to remove %s", path);
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to upload %s, keeping file", path);
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "upload_all_logs: processed %d files", count);
}
