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

/* ── Midea packet constants ─────────────────────────────── */
#define MIDEA_SOF           0xAAU
#define MIDEA_DEVTYPE       0xACU

/* Response type codes (pkt[10] in AC→dongle direction) */
#define RSP_POWER           0xA0U  /* response to QueryPowerData  */
#define RSP_STATUS          0xA1U  /* response to QueryStateData  */

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

/* ── Packet builder ─────────────────────────────────────────
 *
 *  Full Midea UART frame:
 *    [0]     0xAA         SOF
 *    [1]     LEN          bytes from [1] to end (inclusive)
 *    [2]     0xAC         Device type
 *    [3]     0x00         Flags (dongle → AC direction)
 *    [4..7]  0x00 x4      Reserved
 *    [8]     0x00
 *    [9]     0x03         Fixed in TX direction
 *    [10..]  PAYLOAD      CMD byte + data bytes
 *    [n-1]   CRC8         over bytes [2..n-2]
 *    [n]     CHECKSUM     (0 - sum(bytes[1..n-1])) & 0xFF
 */
static int midea_build_packet(uint8_t *out,
                              const uint8_t *payload, size_t payload_len)
{
    uint8_t pkt_len = (uint8_t)(9 + payload_len + 2);

    int i = 0;
    out[i++] = MIDEA_SOF;
    out[i++] = pkt_len;
    out[i++] = MIDEA_DEVTYPE;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x00;
    out[i++] = 0x03;    /* TX direction marker */

    memcpy(&out[i], payload, payload_len);
    i += payload_len;
    {
        uint8_t crc8 = midea_crc8(&out[2], i - 2);
        out[i++] = crc8;
    }

    uint32_t sum = 0;
    for (int j = 1; j < i; j++) sum += out[j];
    out[i++] = (uint8_t)((0x100 - (sum & 0xFF)) & 0xFF);

    return i;
}

/* ── Query payloads ─────────────────────────────────────────
 *  Exact bytes from dudanov/MideaUART StatusData.h:
 *
 *  QueryStateData: {0x41, 0x81, 0x00, 0xFF, 0x03, 0xFF,
 *                   0x00, 0x02, 0x00×12, 0x03, ID}
 *
 *  QueryPowerData: {0x41, 0x21, 0x01, 0x44, 0x00×17, 0x04, ID}
 *    ↑ key: byte[1]=0x21, byte[2]=0x01, byte[3]=0x44
 *      these three tell the AC to return power meter data
 */
static const uint8_t QUERY_STATE_PAYLOAD[] = {
    0x41, 0x81, 0x00, 0xFF, 0x03, 0xFF, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x00
};

static const uint8_t QUERY_POWER_PAYLOAD[] = {
    0x41, 0x21, 0x01, 0x44, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00
};

/* ── Response validation ────────────────────────────────────
 *
 *  Fixed checks (wrong in previous version):
 *    ✗ pkt[9]  == 0x03  →  that's only in TX direction; RX has 0x04 / 0x05
 *    ✗ pkt[10] == 0x41  →  that's the request CMD; responses use 0xA0, 0xA1...
 */
static bool midea_validate_response(const uint8_t *pkt, int len)
{
    if (len < 12)                    return false;
    if (pkt[0] != MIDEA_SOF)         return false;
    if (pkt[2] != MIDEA_DEVTYPE)     return false;
    if ((int)(pkt[1] + 1) > len)     return false; /* LEN byte sanity */
    return true;
}

/* ── Parse power usage from 0xA0 response ───────────────────
 *
 *  Power response layout (verified from your captured bytes):
 *    pkt[10] = 0xA0   ← response type for QueryPowerData
 *    pkt[11] = power high byte
 *    pkt[12] = power mid  byte
 *    pkt[13] = power low  byte
 *
 *  Example from your log:
 *    A0 09 20 65 → raw = 0x092065 = 598117 → 598117/10 = 59811 Wh ≈ 59.8 kWh
 *
 *  Returns cumulative Wh, or -1.0f if wrong response type / too short.
 */
static float midea_parse_power(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return -1.0f;
    if (pkt[10] != RSP_POWER)               return -1.0f;
    if (len < 14)                           return -1.0f;

    uint32_t raw = ((uint32_t)pkt[11] << 16)
                 | ((uint32_t)pkt[12] << 8)
                 |  (uint32_t)pkt[13];

    return (float)raw / 10.0f;  /* Wh */
}

/* ── Parse indoor temperature from 0xA1 response ────────────
 *
 *  Verified from your log:
 *    pkt[23] = 0x61 = 97 → (97 - 50) / 2.0 = 23.5°C  ✓
 *    pkt[25] bit0 = half-degree flag (+0.5°C if set)
 */
static float midea_parse_indoor_temp(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return -99.0f;
    if (pkt[10] != RSP_STATUS)              return -99.0f;
    if (len < 26)                           return -99.0f;

    float temp = (pkt[23] - 50) / 2.0f;
    if (pkt[25] & 0x0F) temp += 0.5f;      /* half-degree bit */
    return temp;
}

/* ── Parse outdoor temperature from 0xA1 response ───────────
 *
 *  Verified from your log:
 *    pkt[24] = 0xFF → outdoor not available (AC off)
 *    Normal range: 0x00..0xFE → (byte - 50) / 2.0
 */
static float midea_parse_outdoor_temp(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return -99.0f;
    if (pkt[10] != RSP_STATUS)              return -99.0f;
    if (len < 25)                           return -99.0f;
    if (pkt[24] == 0xFF)                    return -99.0f; /* not available */

    return (pkt[24] - 50) / 2.0f;
}

/* ── Parse power on/off from 0xA1 response ──────────────────
 *
 *  StatusData.h m_getPower() → m_getValue(1, 1) = byte[1] bit0
 *  In full packet: frame byte[1] = pkt[11]
 */
static bool midea_parse_power_on(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return false;
    if (pkt[10] != RSP_STATUS)              return false;
    if (len < 12)                           return false;

    return (pkt[11] & 0x01) != 0;
}

/* ── Parse mode from 0xA1 response ─────────────────────────
 *
 *  StatusData.h getRawMode() → m_getValue(2, 7, 5) = byte[2] bits[7:5]
 *  In full packet: frame byte[2] = pkt[12]
 */
static const char *midea_parse_mode(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len) || pkt[10] != RSP_STATUS || len < 13)
        return "UNKNOWN";

    static const char *modes[] = {
        "AUTO", "AUTO", "COOL", "DRY", "HEAT", "FAN_ONLY", "?", "?"
    };
    return modes[(pkt[12] >> 5) & 0x07];
}

/* ── Parse target temp from 0xA1 response ───────────────────
 *
 *  StatusData.h getTargetTemp() → byte[2] bits[3:0] + 16
 *  frame byte[2] = pkt[12]
 *  Half-degree bit in pkt[23] bit4 (same byte as indoor temp)
 */
static float midea_parse_target_temp(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len) || pkt[10] != RSP_STATUS || len < 24)
        return -99.0f;

    float temp = (float)((pkt[12] & 0x0F) + 16);
    if (pkt[23] & 0x10) temp += 0.5f;
    return temp;
}

/* ── Find 0xAA start byte in buffer ────────────────────────*/
static int midea_find_start(const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        if (buf[i] == MIDEA_SOF) return i;
    return -1;
}

/* ── Main task ──────────────────────────────────────────────*/
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

    uint8_t *rx_buf = (uint8_t *) malloc(BUF_SIZE);
    uint8_t  pkt_state[64], pkt_power[64];

    int len_state = midea_build_packet(pkt_state, QUERY_STATE_PAYLOAD,
                                       sizeof(QUERY_STATE_PAYLOAD));
    int len_power = midea_build_packet(pkt_power, QUERY_POWER_PAYLOAD,
                                       sizeof(QUERY_POWER_PAYLOAD));

    ESP_LOGI(TAG, "=== Midea AC UART monitor ready ===");
    ESP_LOG_BUFFER_HEX("QueryState TX", pkt_state, len_state);
    ESP_LOG_BUFFER_HEX("QueryPower TX", pkt_power, len_power);

    bool query_power = false;

    while (1) {
        /* ── Send query ──────────────────────────────── */
        if (query_power) {
            uart_write_bytes(ECHO_UART_PORT_NUM, pkt_power, len_power);
            ESP_LOGI(TAG, "TX → QueryPower");
        } else {
            uart_write_bytes(ECHO_UART_PORT_NUM, pkt_state, len_state);
            ESP_LOGI(TAG, "TX → QueryState");
        }

        /* ── Read response ───────────────────────────── */
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, rx_buf,
                                  BUF_SIZE - 1, 500 / portTICK_PERIOD_MS);

        if (len <= 0) {
            ESP_LOGW(TAG, "No response from AC unit");
            query_power = !query_power;
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOG_BUFFER_HEX("RX raw", rx_buf, len);

        /* ── Scan all packets in the buffer ──────────── */
        int offset = 0;
        while (offset < len) {
            /* Find next 0xAA */
            int start = -1;
            for (int i = offset; i < len; i++) {
                if (rx_buf[i] == MIDEA_SOF) { start = i; break; }
            }
            if (start < 0) break;

            const uint8_t *pkt     = &rx_buf[start];
            int            pkt_avail = len - start;

            /* Need at least 2 bytes to read LEN field */
            if (pkt_avail < 2) break;

            int expected_total = pkt[1] + 1;  /* LEN + AA byte */

            if (pkt_avail < expected_total) {
                /* Incomplete packet – would need to read more bytes */
                ESP_LOGW(TAG, "Incomplete packet at offset %d "
                         "(have %d, need %d)", start, pkt_avail, expected_total);
                break;
            }

            if (!midea_validate_response(pkt, expected_total)) {
                ESP_LOGW(TAG, "Invalid packet at offset %d, "
                         "type=0x%02X", start, pkt[10]);
                offset = start + 1;
                continue;
            }

            /* ── Decode by response type ──────────────── */
            uint8_t rsp_type = pkt[10];
            ESP_LOGI(TAG, "── Packet type 0x%02X (%d bytes) ────", rsp_type, expected_total);

            if (rsp_type == RSP_POWER) {
                float power = midea_parse_power(pkt, expected_total);
                if (power >= 0.0f)
                    ESP_LOGI(TAG, "  Power usage : %.1f Wh (%.3f kWh)",
                             power, power / 1000.0f);

            } else if (rsp_type == RSP_STATUS) {
                ESP_LOGI(TAG, "  Power on    : %s",
                         midea_parse_power_on(pkt, expected_total) ? "YES" : "NO");
                ESP_LOGI(TAG, "  Mode        : %s",
                         midea_parse_mode(pkt, expected_total));

                float t_set = midea_parse_target_temp(pkt, expected_total);
                float t_in  = midea_parse_indoor_temp(pkt, expected_total);
                float t_out = midea_parse_outdoor_temp(pkt, expected_total);

                if (t_set > -90.0f) ESP_LOGI(TAG, "  Target temp : %.1f °C", t_set);
                if (t_in  > -90.0f) ESP_LOGI(TAG, "  Indoor temp : %.1f °C", t_in);
                if (t_out > -90.0f) ESP_LOGI(TAG, "  Outdoor temp: %.1f °C", t_out);
                else                ESP_LOGI(TAG, "  Outdoor temp: N/A");

            } else {
                ESP_LOGI(TAG, "  (unhandled response type 0x%02X)", rsp_type);
            }

            ESP_LOGI(TAG, "────────────────────────────────────");
            offset = start + expected_total;
        }

        query_power = !query_power;
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}