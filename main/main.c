#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sync.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "mqtt_client.h"

/* ==== WIFI CONFIG ==== */

#define WIFI_SSID "TP-Link_BC26" // TP-Link SSID
#define WIFI_PASS "15723529"

// Mosquitto op laptop (TP-Link netwerk). Aanpassen indien IP wijzigt.
#define MQTT_URI "mqtt://192.168.1.100:1883"

/* ==== EVENT BITS ==== */

#define WIFI_CONNECTED_BIT BIT0

static const char *TAG = "KLSTR_FIXTURE";

static EventGroupHandle_t s_wifi_event_group;

/* ==== GLOBAL STATE ==== */

static esp_mqtt_client_handle_t s_mqtt = NULL;

// Unieke, stabiele ID op basis van MAC (bv. "esp-24A160ABCDEF")
static char g_device_id[32] = {0};

// Toegewezen fixture ID (bv. "FX-001"), krijgt waarde via provisioning
static char g_fixture_id[32] = {0};

// Flag: al provisioning-request verstuurd?
static bool g_provisioning_requested = false;

/* ==== HELPERS ==== */

static void load_fixture_id_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("klstr", NVS_READONLY, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "No NVS namespace yet (err=%s)", esp_err_to_name(err));
        return;
    }

    size_t len = sizeof(g_fixture_id);
    err = nvs_get_str(nvs, "fixture_id", g_fixture_id, &len);
    nvs_close(nvs);

    if (err == ESP_OK && g_fixture_id[0] != '\0')
    {
        ESP_LOGI(TAG, "Loaded fixture_id from NVS: %s", g_fixture_id);
    }
    else
    {
        g_fixture_id[0] = '\0';
        ESP_LOGI(TAG, "No fixture_id stored in NVS");
    }
}

static void save_fixture_id_to_nvs(const char *fixture_id)
{
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("klstr", NVS_READWRITE, &nvs));
    ESP_ERROR_CHECK(nvs_set_str(nvs, "fixture_id", fixture_id));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);
    ESP_LOGI(TAG, "Saved fixture_id to NVS: %s", fixture_id);
}

static void build_device_id(void)
{
    uint8_t mac[6];

#if CONFIG_SYNC_ROLE_MASTER
    // Master heeft altijd vaste ID
    snprintf(g_device_id, sizeof(g_device_id), "esp-master");
    ESP_LOGI(TAG, "Device ID (MASTER): %s", g_device_id);
#else
    // Slaves gebruiken MAC
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
    snprintf(g_device_id, sizeof(g_device_id),
             "esp-%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Device ID (SLAVE): %s", g_device_id);
#endif
}

/* ==== WIFI ==== */

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WiFi disconnected, retrying");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WiFi connected");
    }
    else
    {
        ESP_LOGE(TAG, "WiFi connect failed");
    }
}

/* ==== MQTT PUBLISH HELPERS ==== */

static void mqtt_publish_status(const char *status)
{
    if (!s_mqtt || g_fixture_id[0] == '\0')
        return;

    char topic[128];
    snprintf(topic, sizeof(topic), "fixture/%s/status", g_fixture_id);

    int mid = esp_mqtt_client_publish(s_mqtt, topic, status, 0, 1, 0);
    ESP_LOGI(TAG, "PUB %s => %s (mid=%d)", topic, status, mid);
}

static void mqtt_publish_result(const char *json)
{
    if (!s_mqtt || g_fixture_id[0] == '\0')
        return;

    char topic[128];
    snprintf(topic, sizeof(topic), "fixture/%s/result", g_fixture_id);

    int mid = esp_mqtt_client_publish(s_mqtt, topic, json, 0, 1, 0);
    ESP_LOGI(TAG, "PUB %s => %s (mid=%d)", topic, json, mid);
}

static void mqtt_send_announce(void)
{
    if (!s_mqtt || g_device_id[0] == '\0')
        return;

    const char *topic = "klstr/devices/announce";

    // Simpel JSON-pakket
    char payload[256];
    snprintf(payload, sizeof(payload),
             "{\"deviceId\":\"%s\"}", g_device_id);

    int mid = esp_mqtt_client_publish(s_mqtt, topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "ANNOUNCE mid=%d: %s => %s", mid, topic, payload);
}

/* ==== COMMAND HANDLER (NA PROVISIONING) ==== */

static void handle_command(const char *topic, const char *payload, int len)
{
    if (g_fixture_id[0] == '\0')
    {
        ESP_LOGW(TAG, "Received command but fixture_id not set yet");
        return;
    }

    char expected_prefix[128];
    snprintf(expected_prefix, sizeof(expected_prefix),
             "fixture/%s/cmd/", g_fixture_id);

    size_t base_len = strlen(expected_prefix);

    if (strncmp(topic, expected_prefix, base_len) != 0)
    {
        ESP_LOGW(TAG, "Ignoring topic %s", topic);
        return;
    }

    const char *cmd = topic + base_len;

    char buf[256];
    int copy_len = (len < (int)sizeof(buf) - 1) ? len : (int)sizeof(buf) - 1;
    memcpy(buf, payload, copy_len);
    buf[copy_len] = '\0';

    ESP_LOGI(TAG, "Cmd=%s Payload=\"%s\"", cmd, buf);

    if (strcmp(cmd, "run_selftest") == 0)
    {
        mqtt_publish_status("running_selftest");
        vTaskDelay(pdMS_TO_TICKS(500));
        mqtt_publish_result("{\"selftest\":\"ok\"}");
        mqtt_publish_status("idle");
    }
    else if (strcmp(cmd, "set_time") == 0)
    {
        long long ts = strtoll(buf, NULL, 10);
        if (ts > 0)
        {
            struct timeval tv = {.tv_sec = (time_t)ts, .tv_usec = 0};
            if (settimeofday(&tv, NULL) == 0)
            {
                ESP_LOGI(TAG, "Time set to %lld", ts);
                mqtt_publish_status("time_synced");
            }
            else
            {
                mqtt_publish_status("time_sync_error");
            }
        }
        else
        {
            mqtt_publish_status("time_sync_invalid");
        }
    }
    else if (strcmp(cmd, "get_time") == 0)
    {
        time_t now;
        time(&now);
        char json[96];
        snprintf(json, sizeof(json), "{\"time_unix\":%ld}", (long)now);
        mqtt_publish_result(json);
        ESP_LOGI(TAG, "Reported time: %ld", (long)now);
    }
    else if (strcmp(cmd, "get_sync_time") == 0)
    {
        int64_t t_us = sync_get_time_us();
        bool synced = sync_is_synced();

        char json[128];
        snprintf(json, sizeof(json),
                 "{\"time_us\":%" PRId64 ",\"synced\":%s}",
                 t_us,
                 synced ? "true" : "false");

        mqtt_publish_result(json);
    }

    else
    {
        ESP_LOGW(TAG, "Unknown command: %s", cmd);
    }
}

/* ==== HANDLE CONFIG MESSAGE ==== */

static void handle_config_message(const char *topic, const char *payload, int len)
{
    if (g_device_id[0] == '\0')
        return;

    char expected_topic[128];
    snprintf(expected_topic, sizeof(expected_topic),
             "klstr/devices/%s/config", g_device_id);

    if (strcmp(topic, expected_topic) != 0)
        return;

    // Verwacht iets als: {"fixtureId":"FX-001"}
    char buf[256];
    int copy_len = (len < (int)sizeof(buf) - 1) ? len : (int)sizeof(buf) - 1;
    memcpy(buf, payload, copy_len);
    buf[copy_len] = '\0';

    ESP_LOGI(TAG, "Config payload: %s", buf);

    const char *key = "\"fixtureId\":\"";
    char *p = strstr(buf, key);
    if (!p)
    {
        ESP_LOGW(TAG, "No fixtureId in config");
        return;
    }
    p += strlen(key);
    char *end = strchr(p, '"');
    if (!end)
    {
        ESP_LOGW(TAG, "Malformed fixtureId field");
        return;
    }

    size_t id_len = end - p;
    if (id_len == 0 || id_len >= sizeof(g_fixture_id))
    {
        ESP_LOGW(TAG, "fixtureId length invalid");
        return;
    }

    memcpy(g_fixture_id, p, id_len);
    g_fixture_id[id_len] = '\0';

    save_fixture_id_to_nvs(g_fixture_id);

    // Nu subscriben op command-topic
    char cmd_topic[128];
    snprintf(cmd_topic, sizeof(cmd_topic), "fixture/%s/cmd/#", g_fixture_id);
    int msg_id = esp_mqtt_client_subscribe(s_mqtt, cmd_topic, 1);
    ESP_LOGI(TAG, "Subscribed to %s (msg_id=%d)", cmd_topic, msg_id);

    mqtt_publish_status("online");
}

/* ==== MQTT EVENT HANDLER ==== */

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");

        if (g_fixture_id[0] != '\0')
        {
            // Bekende fixture: direct subscriben op commands
            char cmd_topic[128];
            snprintf(cmd_topic, sizeof(cmd_topic),
                     "fixture/%s/cmd/#", g_fixture_id);
            esp_mqtt_client_subscribe(event->client, cmd_topic, 1);
            esp_mqtt_client_subscribe(event->client, "fixture/discovery", 1);
            ESP_LOGI(TAG, "Subscribed to %s", cmd_topic);
            ESP_LOGI(TAG, "Subscribed to fixture/discovery");

            mqtt_publish_status("online");
        }
        else
        {
            // Nieuwe / nog niet geprovisioneerde fixture
            char cfg_topic[128];
            snprintf(cfg_topic, sizeof(cfg_topic),
                     "klstr/devices/%s/config", g_device_id);
            esp_mqtt_client_subscribe(event->client, cfg_topic, 1);
            ESP_LOGI(TAG, "Subscribed to %s for config", cfg_topic);

            mqtt_send_announce();
            g_provisioning_requested = true;
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_DATA:
    {
        char topic[256];
        int tlen = (event->topic_len < (int)sizeof(topic) - 1)
                       ? event->topic_len
                       : (int)sizeof(topic) - 1;
        memcpy(topic, event->topic, tlen);
        topic[tlen] = '\0';
        if (strcmp(topic, "fixture/discovery") == 0)
        {
            if (g_fixture_id[0] != '\0')
            {
                mqtt_publish_status("online");
                ESP_LOGI(TAG, "Responded to discovery request");
                break;
            }
            else
            {
                // Niet geprovisioneerd, negeer
                ESP_LOGI(TAG, "Ignoring discovery request, no fixture_id set");
                break;
            }
        }
        ESP_LOGI(TAG, "MQTT DATA topic=%s len=%d", topic, event->data_len);

        // Eerst checken of dit een config-bericht is
        handle_config_message(topic, event->data, event->data_len);

        // Dan pas commands (alleen als fixture_id gezet is)
        handle_command(topic, event->data, event->data_len);
        break;
    }

    default:
        break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    mqtt_event_handler_cb((esp_mqtt_event_handle_t)event_data);
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.client_id = g_device_id,
        .session.keepalive = 30,
        .network.disable_auto_reconnect = false,
    };

    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_mqtt,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
}

/* ==== MAIN ==== */

void app_main(void)
{
#if CONFIG_SYNC_ROLE_MASTER
    ESP_ERROR_CHECK(nvs_flash_erase());
#endif
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "Booting KLSTR fixture");
    build_device_id();
    load_fixture_id_from_nvs();
    // Als er nog geen fixture_id is → gebruik device_id
    if (g_fixture_id[0] == '\0')
    {
        strncpy(g_fixture_id, g_device_id, sizeof(g_fixture_id));
        g_fixture_id[sizeof(g_fixture_id) - 1] = '\0';
        save_fixture_id_to_nvs(g_fixture_id);
        ESP_LOGI(TAG, "Auto-set fixture_id = %s", g_fixture_id);
    }
    wifi_init_sta();
    mqtt_start();

    // Start de UART/MCPWM sync
    sync_init();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
