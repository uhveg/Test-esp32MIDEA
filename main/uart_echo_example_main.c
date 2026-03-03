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
#define MIDEA_SOF       0xAAU
#define MIDEA_DEVTYPE   0xACU
#define MIDEA_FLAGS     0x00U   /* 0x00 = dongle→AC, 0xA0 = AC→dongle */

/* ── CRC-8 (poly 0x31, init 0x00) ──────────────────────── */
static uint8_t midea_crc8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ── Packet builder ─────────────────────────────────────────
 *
 *  Full Midea UART frame layout:
 *    [0]     0xAA         SOF
 *    [1]     LEN          Total packet length (byte[1] onward incl. CS)
 *    [2]     0xAC         Device type: Air Conditioner
 *    [3]     FLAGS        0x00 = dongle→AC
 *    [4..7]  0x00 x4      Reserved
 *    [8..9]  0x00 0x03    Message type indicator
 *    [10..]  PAYLOAD      CMD byte + data bytes (from FrameData)
 *    [n-1]   CRC8         CRC over bytes [2]..[n-2]
 *    [n]     CHECKSUM     (0 - sum of bytes[1..n-1]) & 0xFF
 *
 *  `payload` = the raw bytes array from the FrameData constructor
 *  (already includes CMD byte + data, CRC8 is appended here)
 */
static int midea_build_packet(uint8_t *out, const uint8_t *payload, size_t payload_len)
{
    /* LEN = 9 fixed header bytes after LEN + payload + CRC8 + CS */
    uint8_t pkt_len = (uint8_t)(9 + payload_len + 2);
    ESP_LOGI(TAG, "Building packet: LEN=%d, payload_len=%d", pkt_len, payload_len);

    int i = 0;
    out[i++] = MIDEA_SOF;      /* [0]   0xAA */
    out[i++] = pkt_len;        /* [1]   LEN  */
    out[i++] = MIDEA_DEVTYPE;  /* [2]   0xAC */
    out[i++] = MIDEA_FLAGS;    /* [3]   0x00 */
    out[i++] = 0x00;           /* [4]         */
    out[i++] = 0x00;           /* [5]         */
    out[i++] = 0x00;           /* [6]         */
    out[i++] = 0x00;           /* [7]         */
    out[i++] = 0x00;           /* [8]         */
    out[i++] = 0x03;           /* [9]   0x03  */

    memcpy(&out[i], payload, payload_len);
    i += payload_len;
    ESP_LOGI(TAG, "Payload copied, i=%d", i);

    /* CRC8 over [2 .. i-1] */
    {
        uint8_t crc = midea_crc8(&out[2], i - 2);
        out[i++] = crc;
    }
    ESP_LOGI(TAG, "CRC8 calculated, i=%d", i);

    /* Checksum = (0 - sum(out[1..i-1])) & 0xFF */
    uint32_t sum = 0;
    for (int j = 1; j < i; j++) sum += out[j];
    out[i++] = (uint8_t)((0x100 - (sum & 0xFF)) & 0xFF);
    ESP_LOGI(TAG, "Checksum calculated, i=%d", i);

    return i;
}

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

/* ── QueryPowerData payload ─────────────────────────────────
 *  Exact bytes from dudanov/MideaUART StatusData.h:
 *    QueryPowerData() → {0x41, 0x21, 0x01, 0x44, 0x00 × 17,
 *                        0x04, ID}
 *  Key difference from QueryStateData: byte[1]=0x21, byte[2]=0x01,
 *  byte[3]=0x44 — this is what tells the AC to return power data.
 */
static const uint8_t QUERY_POWER_PAYLOAD[] = {
    0x41, 0x21, 0x01, 0x44, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00   /* 0x00 = ID placeholder */
};

/* ── Response validation ────────────────────────────────── */
static bool midea_validate_response(const uint8_t *pkt, int len)
{
    if (len < 12)                    return false;
    if (pkt[0] != MIDEA_SOF)         return false;
    if (pkt[2] != MIDEA_DEVTYPE)     return false;
    if (pkt[9] != 0x03)              return false;
    if (pkt[10] != 0x41)             return false; /* CMD echo */
    return true;
}

/* ── Parse power usage ──────────────────────────────────────
 *
 *  Full packet byte map (0-indexed):
 *    [0]  AA
 *    [1]  LEN
 *    [2]  AC
 *    [3]  A0  (from appliance)
 *    [4..9]   header bytes
 *    [10] 0x41  CMD echo
 *    [11] flags echo
 *    [12..] status data payload
 *
 *  Power bytes in the response frame (data relative to pkt[10]):
 *    frame[11] = pkt[21]  power high byte
 *    frame[12] = pkt[22]  power mid byte
 *    frame[13] = pkt[23]  power low byte
 *
 *    power_Wh = (pkt[21]<<16 | pkt[22]<<8 | pkt[23]) / 10.0f
 *
 *  Reference: dudanov/MideaUART StatusData.cpp getPowerUsage()
 */
static float midea_parse_power(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return -1.0f;
    if (len < 24) return -1.0f;

    uint32_t raw = ((uint32_t)pkt[21] << 16)
                 | ((uint32_t)pkt[22] << 8)
                 |  (uint32_t)pkt[23];

    return (float)raw / 10.0f;  /* Wh */
}

/* ── Parse indoor temperature ───────────────────────────────
 *
 *  Frame byte[11] (= pkt[21]) holds indoor temp:
 *    indoor_temp = (pkt[21] - 50) / 2.0f  (in Celsius)
 *  Frame byte[15] bit0 (= pkt[25]) adds +0.5°C if set.
 *
 *  Reference: dudanov/MideaUART StatusData.cpp getIndoorTemp()
 */
static float midea_parse_indoor_temp(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return -99.0f;
    if (len < 26) return -99.0f;

    float temp = (pkt[21] - 50) / 2.0f;
    if (pkt[25] & 0x01) temp += 0.5f;
    return temp;
}

/* ── Parse outdoor temperature ─────────────────────────────
 *
 *  Frame byte[12] (= pkt[22]):
 *    outdoor_temp = (pkt[22] - 50) / 2.0f
 *
 *  Reference: dudanov/MideaUART StatusData.cpp getOutdoorTemp()
 */
static float midea_parse_outdoor_temp(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return -99.0f;
    if (len < 23) return -99.0f;

    return (pkt[22] - 50) / 2.0f;
}

/* ── Parse power on/off status ──────────────────────────────
 *
 *  Frame byte[1] bit0 (= pkt[11] & 0x01)
 *  Reference: StatusData.h m_getPower()
 */
static bool midea_parse_power_on(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len)) return false;
    if (len < 12) return false;
    return (pkt[11] & 0x01) != 0;
}

/* ── Parse mode ─────────────────────────────────────────────
 *
 *  Frame byte[2] bits[7:5] (= (pkt[12] >> 5) & 0x07)
 *    0=OFF/AUTO, 1=AUTO, 2=COOL, 3=DRY, 4=HEAT, 5=FAN_ONLY
 */
static const char* midea_parse_mode(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len) || len < 13) return "UNKNOWN";
    static const char* modes[] = {"AUTO","AUTO","COOL","DRY","HEAT","FAN_ONLY","?","?"};
    return modes[(pkt[12] >> 5) & 0x07];
}

/* ── Parse target temperature ───────────────────────────────
 *
 *  Frame byte[2] bits[3:0] (= pkt[12] & 0x0F) + 16
 *  Frame byte[13] bit5 (= pkt[23] & 0x10) adds +0.5
 *
 *  Reference: StatusData.cpp getTargetTemp()
 */
static float midea_parse_target_temp(const uint8_t *pkt, int len)
{
    if (!midea_validate_response(pkt, len) || len < 24) return -99.0f;
    float temp = (float)((pkt[12] & 0x0F) + 16);
    if (pkt[23] & 0x10) temp += 0.5f;
    return temp;
}

/* ── Find start of a valid Midea packet in buffer ─────────── */
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
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    uint8_t *rx_buf = (uint8_t *) malloc(BUF_SIZE);
    uint8_t  tx_buf[64];
    int      tx_len;

    /* Pre-build both query packets */
    uint8_t pkt_state[64], pkt_power[64];
    int     len_state = midea_build_packet(pkt_state, QUERY_STATE_PAYLOAD, sizeof(QUERY_STATE_PAYLOAD));
    int     len_power = midea_build_packet(pkt_power, QUERY_POWER_PAYLOAD, sizeof(QUERY_POWER_PAYLOAD));

    ESP_LOGI(TAG, "=== Midea AC UART monitor ready ===");
    esp_log_buffer_hex("QueryState packet", pkt_state, len_state);
    esp_log_buffer_hex("QueryPower packet", pkt_power, len_power);

    /* Alternate between state and power queries */
    bool query_power = false;

    while (1) {
        /* ── Send query ────────────────────────────────── */
        if (query_power) {
            uart_write_bytes(ECHO_UART_PORT_NUM, pkt_power, len_power);
            ESP_LOGI(TAG, "TX → QueryPower");
        } else {
            uart_write_bytes(ECHO_UART_PORT_NUM, pkt_state, len_state);
            ESP_LOGI(TAG, "TX → QueryState");
        }
        query_power = !query_power;

        /* ── Read response ─────────────────────────────── */
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, rx_buf,
                                  BUF_SIZE - 1,
                                  500 / portTICK_PERIOD_MS);

        if (len <= 0) {
            ESP_LOGW(TAG, "No response from AC unit");
        } else {
            esp_log_buffer_hex("RX raw", rx_buf, len);

            /* Locate packet start (skip any noise) */
            int start = midea_find_start(rx_buf, len);
            if (start < 0) {
                ESP_LOGW(TAG, "No 0xAA start byte found");
            } else {
                const uint8_t *pkt = &rx_buf[start];
                int pkt_len        = len - start;

                if (!midea_validate_response(pkt, pkt_len)) {
                    ESP_LOGW(TAG, "Response failed validation");
                } else {
                    ESP_LOGI(TAG, "── AC Status ────────────────────");

                    /* Power on/off */
                    ESP_LOGI(TAG, "  Power  : %s",
                             midea_parse_power_on(pkt, pkt_len) ? "ON" : "OFF");

                    /* Mode */
                    ESP_LOGI(TAG, "  Mode   : %s",
                             midea_parse_mode(pkt, pkt_len));

                    /* Temperatures */
                    float t_in  = midea_parse_indoor_temp(pkt, pkt_len);
                    float t_out = midea_parse_outdoor_temp(pkt, pkt_len);
                    float t_set = midea_parse_target_temp(pkt, pkt_len);

                    if (t_in  > -90.0f) ESP_LOGI(TAG, "  Indoor : %.1f °C", t_in);
                    if (t_out > -90.0f) ESP_LOGI(TAG, "  Outdoor: %.1f °C", t_out);
                    if (t_set > -90.0f) ESP_LOGI(TAG, "  Target : %.1f °C", t_set);

                    /* Power usage (only valid in power query response) */
                    float power = midea_parse_power(pkt, pkt_len);
                    if (power >= 0.0f)
                        ESP_LOGI(TAG, "  Usage  : %.1f Wh", power);

                    ESP_LOGI(TAG, "─────────────────────────────────");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}