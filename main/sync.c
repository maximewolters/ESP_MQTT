// sync.c
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#include "sync.h"

// ========================= Config =========================

static const char *TAG = "one_way_sync";

#define UARTX UART_NUM_1
#define UART_BAUD 115200
#define UART_RX_BUF 2048
#define UART_TX_BUF 2048

#define UART_TX_PIN 9
#define UART_RX_PIN 10

#define STROBE_GPIO 6

static bool is_master = false;

// ========================= Framing ========================
// | 'TSYN'(4) | type(1) | len(2 LE) | seq(1) | payload(len) |

#pragma pack(push, 1)
typedef struct
{
    uint8_t magic[4];
    uint8_t type;
    uint16_t len; // payload length
    uint8_t seq;  // per-cycle sequence id
} pkt_hdr_t;
#pragma pack(pop)

enum
{
    // zero-length headers used ONLY to generate/identify SOF captures
    PKT_SYNC_HDR = 10, // M→S marker (t1 on M TX, t2 on S RX)
    PKT_RESP_HDR = 11, // S→M marker (t3 on S TX, t4 on M RX)

    // data packets
    PKT_SYNC_REQ = 20,   // M→S payload: int64_t t1
    PKT_SYNC_RESP = 21,  // S→M payload: { int64_t t2, int64_t t3 }
    PKT_SET_OFFSET = 22, // M→S payload: int64_t offset (optional convenience)
};

static inline void make_hdr(pkt_hdr_t *h, uint8_t type, uint16_t len, uint8_t seq)
{
    h->magic[0] = 'T';
    h->magic[1] = 'S';
    h->magic[2] = 'Y';
    h->magic[3] = 'N';
    h->type = type;
    h->len = len;
    h->seq = seq;
}

static inline bool hdr_ok(const pkt_hdr_t *h)
{
    return h->magic[0] == 'T' && h->magic[1] == 'S' &&
           h->magic[2] == 'Y' && h->magic[3] == 'N';
}

#define UART_BITS_PER_BYTE 10
#define HDR_BYTES (sizeof(pkt_hdr_t))
#define INTERFRAME_GAP_US ((int)((HDR_BYTES * UART_BITS_PER_BYTE * 1000000UL) / UART_BAUD) + 300)

// ========================= UART helpers ========================

static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(UARTX, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UARTX, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UARTX, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static esp_err_t uart_send(uart_port_t u,
                           uint8_t type,
                           uint8_t seq,
                           const void *payload,
                           uint16_t len)
{
    pkt_hdr_t h;
    make_hdr(&h, type, len, seq);
    int w = uart_write_bytes(u, (const char *)&h, sizeof(h));
    if (w != (int)sizeof(h))
        return ESP_FAIL;

    if (len && payload)
    {
        int w2 = uart_write_bytes(u, (const char *)payload, len);
        if (w2 != (int)len)
            return ESP_FAIL;
    }
    return ESP_OK;
}

static int uart_recv(uart_port_t u,
                     uint8_t *payload,
                     size_t pay_cap,
                     pkt_hdr_t *out_hdr,
                     int timeout_ms)
{
    size_t got = 0;
    pkt_hdr_t h;
    int64_t dl = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    // header
    while (got < sizeof(h))
    {
        int to = (int)((dl - esp_timer_get_time()) / 1000);
        if (to <= 0)
            return -1;
        int n = uart_read_bytes(u, ((uint8_t *)&h) + got,
                                sizeof(h) - got, pdMS_TO_TICKS(to));
        if (n > 0)
            got += n;
        else if (n < 0)
            return -1;
    }
    if (!hdr_ok(&h))
        return -2;
    if (h.len > pay_cap)
        return -3;

    // payload
    got = 0;
    while (got < h.len)
    {
        int to = (int)((dl - esp_timer_get_time()) / 1000);
        if (to <= 0)
            return -4;
        int n = uart_read_bytes(u, payload + got,
                                h.len - got, pdMS_TO_TICKS(to));
        if (n > 0)
            got += n;
        else if (n < 0)
            return -4;
    }

    if (out_hdr)
        *out_hdr = h;
    return h.type;
}

// ================= HW capture & mapping ===================

static mcpwm_cap_timer_handle_t cap_timer;
static mcpwm_cap_channel_handle_t cap_tx_ch, cap_rx_ch, cap_strobe_ch;
static double ticks_to_us = 0.0;

typedef struct
{
    uint32_t tick;
    int64_t t_us;
} sync_base_t;

static sync_base_t base0, base1;
static portMUX_TYPE base_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t rx_tick, tx_tick;
static volatile bool rx_seen, tx_seen;
static volatile bool rx_armed, tx_armed;

// strobe capture → update bases
static bool IRAM_ATTR on_cap_strobe(mcpwm_cap_channel_handle_t ch,
                                    const mcpwm_capture_event_data_t *e,
                                    void *u)
{
    (void)ch;
    (void)u;
    sync_base_t b0;
    b0.tick = e->cap_value;
    b0.t_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&base_mux);
    base1 = base0;
    base0 = b0;
    portEXIT_CRITICAL_ISR(&base_mux);
    return true;
}

// RX edge
static bool IRAM_ATTR on_cap_rx(mcpwm_cap_channel_handle_t ch,
                                const mcpwm_capture_event_data_t *e,
                                void *u)
{
    (void)ch;
    (void)u;
    if (!rx_armed)
        return true;
    rx_tick = e->cap_value;
    rx_seen = true;
    rx_armed = false;
    return true;
}

// TX edge
static bool IRAM_ATTR on_cap_tx(mcpwm_cap_channel_handle_t ch,
                                const mcpwm_capture_event_data_t *e,
                                void *u)
{
    (void)ch;
    (void)u;
    if (!tx_armed)
        return true;
    tx_tick = e->cap_value;
    tx_seen = true;
    tx_armed = false;
    return true;
}

// strobe GPIO toggle
static void strobe_cb(void *arg)
{
    (void)arg;
    static bool lvl = false;
    lvl = !lvl;
    gpio_set_level(STROBE_GPIO, lvl);
}

static void capture_init_start(void)
{
    // strobe pin
    ESP_ERROR_CHECK(gpio_config(&(gpio_config_t){
        .pin_bit_mask = 1ULL << STROBE_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    }));
    gpio_set_level(STROBE_GPIO, 0);

    // capture timer @ 80 MHz
    mcpwm_capture_timer_config_t tcfg = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_APB,
        .resolution_hz = 80000000};
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&tcfg, &cap_timer));
    ticks_to_us = 1000000.0 / (double)tcfg.resolution_hz;

    // channels
    mcpwm_capture_channel_config_t c_rx = {
        .gpio_num = UART_RX_PIN,
        .prescale = 1,
        .flags = {.pos_edge = 0, .neg_edge = 1, .pull_up = 1}};
    mcpwm_capture_channel_config_t c_tx = {
        .gpio_num = UART_TX_PIN,
        .prescale = 1,
        .flags = {.pos_edge = 0, .neg_edge = 1, .pull_up = 1}};
    mcpwm_capture_channel_config_t c_st = {
        .gpio_num = STROBE_GPIO,
        .prescale = 1,
        .flags = {.pos_edge = 1, .neg_edge = 0, .pull_up = 0}};

    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &c_rx, &cap_rx_ch));
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &c_tx, &cap_tx_ch));
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &c_st, &cap_strobe_ch));

    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(
        cap_rx_ch, &(mcpwm_capture_event_callbacks_t){.on_cap = on_cap_rx}, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(
        cap_tx_ch, &(mcpwm_capture_event_callbacks_t){.on_cap = on_cap_tx}, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(
        cap_strobe_ch, &(mcpwm_capture_event_callbacks_t){.on_cap = on_cap_strobe}, NULL));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_rx_ch));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_tx_ch));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_strobe_ch));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    // 10 kHz strobe via esp_timer
    const esp_timer_create_args_t ta = {
        .callback = &strobe_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "map_strobe",
    };
    static esp_timer_handle_t strobe_tmr;
    ESP_ERROR_CHECK(esp_timer_create(&ta, &strobe_tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(strobe_tmr, 100)); // 100 µs = 10 kHz

    // init bases
    sync_base_t z;
    z.tick = 0;
    z.t_us = esp_timer_get_time();
    portENTER_CRITICAL(&base_mux);
    base0 = z;
    base1 = z;
    portEXIT_CRITICAL(&base_mux);
}

static inline void arm_rx(void)
{
    rx_seen = false;
    rx_armed = true;
}

static inline void arm_tx(void)
{
    tx_seen = false;
    tx_armed = true;
}

// project captured tick to esp_timer epoch
static inline int64_t project_tick(uint32_t tick)
{
    sync_base_t b0, b1;
    portENTER_CRITICAL(&base_mux);
    b0 = base0;
    b1 = base1;
    portEXIT_CRITICAL(&base_mux);

    int32_t d0 = (int32_t)(tick - b0.tick);
    int32_t d1 = (int32_t)(tick - b1.tick);
    sync_base_t b = ((d0 >= 0 && (d0 <= d1 || d1 < 0)) ? b0 : b1);
    int32_t dt = (int32_t)(tick - b.tick);
    double dus = (double)dt * ticks_to_us;
    int64_t res = (int64_t)(b.t_us + (dus >= 0 ? (dus + 0.5) : (dus - 0.5)));
    return res;
}

static inline bool consume_rx_edge(int64_t *t)
{
    if (!rx_seen)
        return false;
    *t = project_tick(rx_tick);
    rx_seen = false;
    return true;
}

static inline bool consume_tx_edge(int64_t *t)
{
    if (!tx_seen)
        return false;
    *t = project_tick(tx_tick);
    tx_seen = false;
    return true;
}

// ================= Sequence-indexed edge store =================

typedef struct
{
    bool have;
    int64_t t_us;
} seq_edge_t;

static seq_edge_t edges[256];

static inline void store_edge(uint8_t seq, int64_t t)
{
    edges[seq].have = true;
    edges[seq].t_us = t;
}

static inline bool take_edge(uint8_t seq, int64_t *t)
{
    if (!edges[seq].have)
        return false;
    *t = edges[seq].t_us;
    edges[seq].have = false;
    return true;
}

// ================= Globale offset & status =================

static volatile int64_t g_offset_us = 0;
static volatile bool g_have_offset = false;

static inline int64_t synced_time_us_internal(void)
{
    return esp_timer_get_time() - g_offset_us;
}

bool sync_is_synced(void)
{
    return g_have_offset;
}

int64_t sync_get_time_us(void)
{
    if (!g_have_offset)
        return esp_timer_get_time();
    return synced_time_us_internal();
}

int64_t sync_get_offset_us(void)
{
    return g_offset_us;
}

// ========================== MASTER ==============================

static void master_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Role: MASTER. TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);

    uint8_t seq = 0;

    while (1)
    {
        // 1) Marker: stuur SYNC_HDR om t1 op TX te capturen
        arm_tx();
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_HDR, seq, NULL, 0));
        uart_wait_tx_done(UARTX, pdMS_TO_TICKS(2));
        esp_rom_delay_us(INTERFRAME_GAP_US);

        int64_t t1 = 0;
        for (int i = 0; i < 4 && !consume_tx_edge(&t1); ++i)
        {
            vTaskDelay(1);
        }
        if (!t1)
        {
            ESP_LOGW(TAG, "no t1 edge");
            goto next;
        }

        // 2) DATA: stuur SYNC_REQ{t1}
        ESP_ERROR_CHECK(uart_send(UARTX, PKT_SYNC_REQ, seq, &t1, sizeof(t1)));

    next:
        seq++;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 Hz sync-cycles
    }
}

// ========================== SLAVE ==============================

static void slave_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Role: SLAVE. TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);

    uint8_t buf[32];

    while (1)
    {
        // 1) wacht eerst op de marker-header (PKT_SYNC_HDR)
        arm_rx();
        pkt_hdr_t h;
        int typ = uart_recv(UARTX, buf, sizeof(buf), &h, 5000);
        if (typ != PKT_SYNC_HDR)
        {
            ESP_LOGW(TAG, "unexpected type %d (want SYNC_HDR)", typ);
            continue;
        }

        // 2) capture van de RX-edge → t2
        int64_t t2 = 0;
        if (!consume_rx_edge(&t2))
        {
            vTaskDelay(1);
            if (!consume_rx_edge(&t2))
            {
                ESP_LOGW(TAG, "no t2 edge");
                continue;
            }
        }

        uint8_t seq = h.seq;

        // 3) lees nu de data-packet SYNC_REQ{t1} met dezelfde seq
        typ = uart_recv(UARTX, buf, sizeof(buf), &h, 50);
        if (typ != PKT_SYNC_REQ || h.len != 8 || h.seq != seq)
        {
            ESP_LOGW(TAG, "bad SYNC_REQ: type=%d len=%u seq(hdr)=%u, expected type=%d len=8 seq=%u",
                     typ, h.len, h.seq, PKT_SYNC_REQ, seq);
            continue;
        }

        int64_t t1;
        memcpy(&t1, buf, 8);

        // 4) offset = lokale_tijd - master_tijd
        int64_t offset = t2 - t1;

        g_offset_us = offset;
        g_have_offset = true;

        ESP_LOGI(TAG, "New offset: %lld us", (long long)offset);
        ESP_LOGI(TAG, "Synced time now: %lld us",
                 (long long)sync_get_time_us());
    }
}

void sync_init(void)
{
    ESP_LOGI(TAG, "Sync init...");
    capture_init_start();
    uart_init();
    vTaskDelay(pdMS_TO_TICKS(50));

#if CONFIG_SYNC_ROLE_MASTER
    xTaskCreatePinnedToCore(master_task, "sync_master",
                            4096, NULL, 5, NULL, 0);
#elif CONFIG_SYNC_ROLE_SLAVE
    is_master = false;
    xTaskCreatePinnedToCore(slave_task, "sync_slave",
                            4096, NULL, 5, NULL, 1);
#else
#warning "No role set; defaulting to MASTER"
    xTaskCreatePinnedToCore(master_task, "sync_master",
                            4096, NULL, 5, NULL, 0);
#endif
}
