#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/* ── Pins & UART config ─────────────────────────────────── */
#define ECHO_TEST_TXD           21
#define ECHO_TEST_RXD           20
#define ECHO_TEST_RTS           (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS           (UART_PIN_NO_CHANGE)
#define ECHO_UART_PORT_NUM      UART_NUM_0
#define ECHO_UART_BAUD_RATE     9600
#define ECHO_TASK_STACK_SIZE    4096
#define BUF_SIZE                1024

static const char *TAG = "MIDEA_AC";

#define MIDEA_SOF    0xAAU
#define MIDEA_DEVTYPE 0xACU

/* ── Response type codes sent by the AC ────────────────────
 *
 *  0xA0  Power usage data       (broadcast + response to QueryPower)
 *  0xA1  Full status            (response to QueryState)
 *  0xA2  Status broadcast       (AC sends spontaneously, same format as A1)
 *  0xA3  Status broadcast       (same)
 *  0xA5  Extended status        (timers, fan detail, etc.)
 *  0xA6  Status broadcast       (same as A2/A3)
 *  0xB5  Capabilities report    (AC sends when it thinks dongle reconnected)
 */
#define RSP_POWER    0xA0U
#define RSP_STATUS_1 0xA1U
#define RSP_STATUS_2 0xA2U
#define RSP_STATUS_3 0xA3U
#define RSP_STATUS_5 0xA5U
#define RSP_STATUS_6 0xA6U
#define RSP_CAPS     0xB5U

/* ── CRC-8 (poly 0x31, init 0x00) ──────────────────────── */
static uint8_t midea_crc8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ── Packet builder ─────────────────────────────────────── */
static int midea_build_packet(uint8_t *out, uint8_t flags, const uint8_t *payload, size_t payload_len)
{
    uint8_t pkt_len = (uint8_t)(9 + payload_len + 2);
    int i = 0;
    out[i++] = MIDEA_SOF;
    out[i++] = pkt_len;
    out[i++] = MIDEA_DEVTYPE;
    out[i++] = flags;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x03;
    memcpy(&out[i], payload, payload_len);
    i += payload_len;
    uint8_t crc8 = midea_crc8(&out[2], i - 2);
    out[i++] = crc8;
    uint32_t sum = 0;
    for (int j = 1; j < i; j++) sum += out[j];
    out[i++] = (uint8_t)((0x100 - (sum & 0xFF)) & 0xFF);
    return i;
}

/* ── Network Status Notification ───────────────────────────
 *
 *  This is the 0x0D command. It tells the AC:
 *    "The WiFi dongle is connected to the network."
 *  Without it, the AC periodically sends B5 capability probes
 *  (visible in your log) and may remove the WiFi icon from its display.
 *  Must be sent at least every 2 minutes.
 *
 *  Payload: {0x0D, 0x01, [IP4 bytes], [router_IP4], 0x00, ssid_len, ...}
 *  A minimal all-zeros version is accepted by most models.
 */
static const uint8_t NOTIFY_NETWORK_PAYLOAD[] = {
    0x0D,                               /* CMD: network notification  */
    0x01,                               /* connected = true           */
    0x00, 0x00, 0x00, 0x00,             /* dongle IP (0 = unknown ok) */
    0x00, 0x00, 0x00, 0x00,             /* router IP                  */
    0x00,                               /* signal strength            */
    0x00                                /* SSID length = 0            */
};
/* ── QueryStateData payload ─────────────────────────────────
 *  Exact bytes from dudanov/MideaUART StatusData.h:
 *    QueryStateData() → {0x41, 0x81, 0x00, 0xFF, 0x03, 0xFF,
 *                        0x00, 0x02, 0x00 × 12, 0x03, ID}
 *  (ID = rolling counter, 0x00 is fine for one-shot queries)
 */
static const uint8_t QUERY_STATE_PAYLOAD[] = {
    0x41, 0x81, 0x00, 0xFF, 0x03, 0xFF, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x00   /* 0x00 = ID placeholder */
};


/* ── Validation ─────────────────────────────────────────── */
static bool midea_validate(const uint8_t *pkt, int len)
{
    if (len < 12)                          return false;
    if (pkt[0] != MIDEA_SOF)               return false;
    if (pkt[2] != MIDEA_DEVTYPE)           return false;
    if ((int)(pkt[1] + 1) > len)           return false;
    return true;
}

/* ── Is this a status-type response (A1/A2/A3/A5/A6)? ─────
 *
 *  All of these share the same payload layout as A1.
 *  A5 has extra fields starting around pkt[33] (timer/fan detail)
 *  but the core status bytes are identical.
 */
static bool is_status_type(uint8_t t)
{
    return (t == RSP_STATUS_1 || t == RSP_STATUS_2 ||
            t == RSP_STATUS_3 || t == RSP_STATUS_5 ||
            t == RSP_STATUS_6);
}

static bool midea_payload_has_data(const uint8_t *pkt, int len)
{
    /* If indoor temp byte is 0x00, the payload is empty/zeroed.
     * Real indoor temp byte is always >= 0x02 (maps to >= -24°C).
     * A2/A3/A5/A6 broadcasts when AC is off often carry zero payload. */
    if (len < 24) return false;
    return pkt[23] > 0x01;
}

/* ── Parse power on/off ─────────────────────────────────── */
static bool midea_parse_power_on(const uint8_t *pkt, int len)
{
    if (len < 12) return false;
    return (pkt[11] & 0x01) != 0;
}

/* ── Parse mode ─────────────────────────────────────────── */
static const char *midea_parse_mode(const uint8_t *pkt, int len)
{
    if (len < 13) return "?";
    static const char *modes[] = {
        "AUTO", "AUTO", "COOL", "DRY", "HEAT", "FAN_ONLY", "?", "?"
    };
    return modes[(pkt[12] >> 5) & 0x07];
}

/* ── Parse fan speed ────────────────────────────────────── */
static const char *midea_parse_fan(const uint8_t *pkt, int len)
{
    if (len < 14) return "?";
    uint8_t spd = pkt[13] & 0x7F;
    if (spd == 102) return "AUTO";
    if (spd >= 80)  return "HIGH";
    if (spd >= 60)  return "MEDIUM";
    if (spd >= 40)  return "LOW";
    if (spd >= 20)  return "SILENT";
    return "AUTO";
}

/* ── Parse target temperature ───────────────────────────── */
static float midea_parse_target_temp(const uint8_t *pkt, int len)
{
    if (len < 24) return -99.0f;
    float temp = (float)((pkt[12] & 0x0F) + 16);
    if (pkt[23] & 0x10) temp += 0.5f;
    return temp;
}

/* ── Parse indoor temperature ───────────────────────────── */
static float midea_parse_indoor_temp(const uint8_t *pkt, int len)
{
    if (len < 24) return -99.0f;
    float temp = (pkt[23] - 50) / 2.0f;
    if (len >= 26 && (pkt[25] & 0x0F)) temp += 0.5f;
    return temp;
}

/* ── Parse outdoor temperature ──────────────────────────── */
static float midea_parse_outdoor_temp(const uint8_t *pkt, int len)
{
    if (len < 25)        return -99.0f;
    if (pkt[24] == 0xFF) return -99.0f;
    return (pkt[24] - 50) / 2.0f;
}

/* ── Parse power usage from 0xA0 ───────────────────────── */
static float midea_parse_power_energy(const uint8_t *pkt, int len)
{
    if (len < 14 || pkt[10] != RSP_POWER) return -1.0f;
    uint32_t raw = ((uint32_t)pkt[11] << 16)
                 | ((uint32_t)pkt[12] << 8)
                 |  (uint32_t)pkt[13];
    return (float)raw / 10.0f;
}

static float midea_parse_power_watts(const uint8_t *pkt, int len)
{
    if (len < 16 || pkt[10] != RSP_POWER) return -1.0f;
    /* 0x7F7F = not available (unit is off) */
    if (pkt[14] == 0x7F && pkt[15] == 0x7F) return -1.0f;
    uint16_t raw = ((uint16_t)pkt[14] << 8) | pkt[15];
    return (float)raw;  /* Watts */
}

/* ── Parse B5 capabilities ──────────────────────────────────
 *
 *  B5 payload: {0xB5, 0x01, count, [id, len, val, ...], ...}
 *  We just log the key powerCal flag (id=0x10, byte 0x06 bit1).
 *  Full decode requires iterating all capability TLV entries.
 */
static void midea_parse_b5(const uint8_t *pkt, int len)
{
    /* pkt[10]=0xB5, pkt[11]=0x01, pkt[12]=count of items */
    if (len < 13) return;
    ESP_LOGI(TAG, "  [B5] Capabilities packet (%d items)", pkt[12]);

    /* Scan for powerCal capability (ID = 0x10, sub-ID = 0x06) */
    int i = 13;
    while (i + 2 < len - 2) {   /* -2 = skip CRC+CS at tail */
        uint8_t cap_id  = pkt[i];
        uint8_t cap_sub = pkt[i + 1];
        uint8_t cap_val = pkt[i + 2];
        if (cap_id == 0x10 && cap_sub == 0x06) {
            ESP_LOGI(TAG, "  [B5] powerCal (power metering) : %s",
                     (cap_val & 0x01) ? "SUPPORTED" : "not supported");
        }
        i += 3;
    }
}

/* ── Decode and print a single validated packet ─────────── */
static void midea_decode_packet(const uint8_t *pkt, int len)
{
    uint8_t rsp = pkt[10];

    if (rsp == RSP_POWER) {
        float energy = midea_parse_power_energy(pkt, len);
        float watts  = midea_parse_power_watts(pkt, len);
        if (energy >= 0.0f)
            ESP_LOGI(TAG, "[A0] Energy usage : %.1f Wh  (%.3f kWh)", energy, energy / 1000.0f);
        if (watts >= 0.0f)
            ESP_LOGI(TAG, "[A0] Power   : %.0f W", watts);
        else
            ESP_LOGI(TAG, "[A0] Power   : N/A (unit off)");
    } else if (is_status_type(rsp)) {
        /* A1=queried, A2/A3/A6=spontaneous broadcast, A5=extended */
        const char *src = (rsp == RSP_STATUS_1) ? "queried"
                        : (rsp == RSP_STATUS_5) ? "extended"
                        : "broadcast";
        ESP_LOGI(TAG, "[%02X] Status (%s) ─────────────────", rsp, src);
        ESP_LOGI(TAG, "  Power  : %s", midea_parse_power_on(pkt, len) ? "ON" : "OFF");
        ESP_LOGI(TAG, "  Mode   : %s",  midea_parse_mode(pkt, len));
        ESP_LOGI(TAG, "  Fan    : %s",  midea_parse_fan(pkt, len));
        
        if (midea_payload_has_data(pkt, len)) {
            float t_set = midea_parse_target_temp(pkt, len);
            float t_in  = midea_parse_indoor_temp(pkt, len);
            float t_out = midea_parse_outdoor_temp(pkt, len);

            if (t_set > -90.0f) ESP_LOGI(TAG, "  Target : %.1f °C", t_set);
            if (t_in  > -90.0f) ESP_LOGI(TAG, "  Indoor : %.1f °C", t_in);
            if (t_out > -90.0f) ESP_LOGI(TAG, "  Outdoor: %.1f °C", t_out);
            else                ESP_LOGI(TAG, "  Outdoor: N/A (unit off)");
        } else {
            ESP_LOGI(TAG, "  (empty broadcast — no temperature data)");
        }
    } else if (rsp == RSP_CAPS) {
        midea_parse_b5(pkt, len);

    } else {
        /* Log unknown types at DEBUG level so they don't spam */
        ESP_LOGD(TAG, "[%02X] unhandled type (%d bytes)", rsp, len);
        ESP_LOG_BUFFER_HEX("  raw", pkt, len);
    }
}

/* ── Main task ──────────────────────────────────────────── */
static void echo_task(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate  = ECHO_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0,
                                        NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD,
                                 ECHO_TEST_RTS, ECHO_TEST_CTS));

    uint8_t *rx_buf  = (uint8_t *) malloc(BUF_SIZE);
    uint8_t  pkt_notify[32];
    int      len_notify = midea_build_packet(pkt_notify, 0xB2, NOTIFY_NETWORK_PAYLOAD, sizeof(NOTIFY_NETWORK_PAYLOAD));

    ESP_LOGI(TAG, "=== Midea AC passive monitor ready ===");
    ESP_LOGI(TAG, "Listening only — AC sends status every ~2-5 seconds");
    ESP_LOG_BUFFER_HEX("Network notify TX", pkt_notify, len_notify);

    /* Send network notification immediately on boot */
    uart_write_bytes(ECHO_UART_PORT_NUM, pkt_notify, len_notify);
    ESP_LOGI(TAG, "TX → Network notification sent");

    TickType_t last_notify = xTaskGetTickCount();
    TickType_t last_query = xTaskGetTickCount();

    while (1) {
        /* ── Send network notification every 90 seconds ──
         *
         *  This keeps the WiFi icon on the AC display and
         *  stops the AC from flooding us with B5 probes.
         */
        if ((xTaskGetTickCount() - last_notify) > pdMS_TO_TICKS(90000)) {
            uart_write_bytes(ECHO_UART_PORT_NUM, pkt_notify, len_notify);
            ESP_LOGI(TAG, "TX → Network notification (keepalive)");
            last_notify = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - last_query) > pdMS_TO_TICKS(30000)) {
            uint8_t pkt_state[64];
            int len_state = midea_build_packet(pkt_state, 0x00, QUERY_STATE_PAYLOAD, sizeof(QUERY_STATE_PAYLOAD));
            uart_write_bytes(ECHO_UART_PORT_NUM, pkt_state, len_state);
            ESP_LOGI(TAG, "TX → QueryState (TX test)");
            last_query = xTaskGetTickCount();
        }

        /* ── Read whatever the AC is broadcasting ────────
         *
         *  50ms timeout: short enough to catch multi-packet
         *  bursts without busy-waiting.
         */
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, rx_buf, BUF_SIZE - 1, 50 / portTICK_PERIOD_MS);

        if (len <= 0) continue;

        ESP_LOG_BUFFER_HEX("RX", rx_buf, len);

        /* ── Scan all packets in the buffer ─────────────── */
        int offset = 0;
        while (offset < len) {
            /* Find next 0xAA start byte */
            int start = -1;
            for (int i = offset; i < len; i++) {
                if (rx_buf[i] == MIDEA_SOF) { start = i; break; }
            }
            if (start < 0) break;

            const uint8_t *pkt      = &rx_buf[start];
            int            available = len - start;

            if (available < 2) break;

            int expected = pkt[1] + 1;   /* LEN field + the 0xAA byte */

            if (available < expected) {
                /* Incomplete — will catch the rest next read */
                ESP_LOGD(TAG, "Incomplete packet (have %d / need %d)",
                         available, expected);
                break;
            }

            if (!midea_validate(pkt, expected)) {
                offset = start + 1;
                continue;
            }

            midea_decode_packet(pkt, expected);
            offset = start + expected;
        }
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}