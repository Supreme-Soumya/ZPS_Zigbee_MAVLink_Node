/*
 * mavlink_reader.c
 *
 * Minimal MAVLink v1 parser — no external library needed.
 * Extracts GPS, speed, heading and time from the APM 2.8 TELEM port
 * and exposes a gps_snapshot_t struct for end_device.c to send over Zigbee.
 *
 * MAVLink messages parsed:
 *   #2  SYSTEM_TIME         → UTC timestamp
 *   #24 GPS_RAW_INT         → fix, lat, lon, alt, hdop, satellites
 *   #33 GLOBAL_POSITION_INT → lat, lon, alt, heading
 *   #74 VFR_HUD             → groundspeed, heading (fallback)
 *
 * FIX (vs original): The parser state machine (process_byte) previously
 * used static local variables for its state, but was called with a local
 * copy `tmp` of the snapshot inside the task loop. This meant the parser
 * state advanced (consumed bytes) but wrote into a `tmp` that was then
 * discarded when the mutex was released on the next iteration boundary —
 * potentially losing partial-frame state under certain scheduling scenarios.
 *
 * Fix: parser state is now a separate persistent struct (mavlink_parser_t)
 * that lives for the lifetime of the task, completely decoupled from the
 * gps_snapshot_t. The snapshot is updated only on a complete, CRC-valid
 * frame via dispatch_frame(), which takes s_snap directly under the mutex.
 * This makes the data flow explicit and race-free.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "mavlink_reader.h"

static const char *TAG = "MAV_READER";

/* ── Internal snapshot + mutex ───────────────────────────── */
static gps_snapshot_t   s_snap  = {0};
static SemaphoreHandle_t s_mutex = NULL;

/* ═══════════════════════════════════════════════════════════
 * MAVLink v1 frame constants
 * ═══════════════════════════════════════════════════════════ */
#define MAVLINK_STX         0xFE
#define MAVLINK_HDR_LEN     6
#define MAVLINK_CRC_LEN     2
#define MAVLINK_MAX_PAYLOAD 255
#define MAVLINK_MAX_FRAME   (MAVLINK_HDR_LEN + MAVLINK_MAX_PAYLOAD + MAVLINK_CRC_LEN)

#define MSG_SYSTEM_TIME         2
#define MSG_GPS_RAW_INT         24
#define MSG_GLOBAL_POSITION_INT 33
#define MSG_VFR_HUD             74

static const struct { uint8_t id; uint8_t crc_extra; } s_crc_table[] = {
    { MSG_SYSTEM_TIME,         137 },
    { MSG_GPS_RAW_INT,          24 },
    { MSG_GLOBAL_POSITION_INT, 104 },
    { MSG_VFR_HUD,              20 },
};

/* ── CRC-16/MCRF4XX ───────────────────────────────────────── */
static void crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (uint8_t)(*crc & 0xFF);
    tmp ^= (tmp << 4);
    *crc = (*crc >> 8) ^ ((uint16_t)tmp << 8) ^
           ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4);
}

/* ── Little-endian helpers ────────────────────────────────── */
static inline int32_t  le_i32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24);
}
static inline uint32_t le_u32(const uint8_t *p) {
    return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;
}
static inline uint64_t le_u64(const uint8_t *p) {
    return (uint64_t)le_u32(p) | ((uint64_t)le_u32(p+4) << 32);
}
static inline int16_t  le_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0]|(uint16_t)p[1]<<8);
}
static inline uint16_t le_u16(const uint8_t *p) {
    return (uint16_t)p[0]|(uint16_t)p[1]<<8;
}
static inline float    le_f32(const uint8_t *p) {
    float f; memcpy(&f, p, 4); return f;
}

/* ═══════════════════════════════════════════════════════════
 * Message parsers — write directly into s_snap under mutex
 * ═══════════════════════════════════════════════════════════ */

/* #2 SYSTEM_TIME */
static void parse_system_time(const uint8_t *payload)
{
    uint64_t unix_usec = le_u64(payload);
    if (unix_usec == 0) return;

    time_t unix_sec = (time_t)(unix_usec / 1000000ULL);
    struct tm t;
    gmtime_r(&unix_sec, &t);

    int year  = (t.tm_year + 1900) % 10000;
    int month = t.tm_mon + 1;
    int day   = t.tm_mday;
    int hour  = t.tm_hour;
    int min   = t.tm_min;
    int sec   = t.tm_sec;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snap.utc[0]  = '0' + (year  / 1000);
    s_snap.utc[1]  = '0' + (year  / 100 % 10);
    s_snap.utc[2]  = '0' + (year  / 10  % 10);
    s_snap.utc[3]  = '0' + (year  % 10);
    s_snap.utc[4]  = '-';
    s_snap.utc[5]  = '0' + (month / 10);
    s_snap.utc[6]  = '0' + (month % 10);
    s_snap.utc[7]  = '-';
    s_snap.utc[8]  = '0' + (day   / 10);
    s_snap.utc[9]  = '0' + (day   % 10);
    s_snap.utc[10] = 'T';
    s_snap.utc[11] = '0' + (hour  / 10);
    s_snap.utc[12] = '0' + (hour  % 10);
    s_snap.utc[13] = ':';
    s_snap.utc[14] = '0' + (min   / 10);
    s_snap.utc[15] = '0' + (min   % 10);
    s_snap.utc[16] = ':';
    s_snap.utc[17] = '0' + (sec   / 10);
    s_snap.utc[18] = '0' + (sec   % 10);
    s_snap.utc[19] = '\0';
    xSemaphoreGive(s_mutex);
}

/* #24 GPS_RAW_INT */
static void parse_gps_raw_int(const uint8_t *payload)
{
    uint8_t  fix_type = payload[28];
    int32_t  lat_1e7  = le_i32(payload + 8);
    int32_t  lon_1e7  = le_i32(payload + 12);
    int32_t  alt_mm   = le_i32(payload + 16);
    uint16_t eph      = le_u16(payload + 20);
    uint8_t  sats     = payload[29];

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snap.fix_valid  = (fix_type >= 3) ? 1 : 0;
    s_snap.lat        = lat_1e7 / 1e7;
    s_snap.lon        = lon_1e7 / 1e7;
    s_snap.alt_m      = alt_mm  / 1000.0f;
    s_snap.hdop       = (eph == UINT16_MAX) ? 99.99f : eph / 100.0f;
    s_snap.satellites = sats;
    xSemaphoreGive(s_mutex);
}

/* #33 GLOBAL_POSITION_INT */
static void parse_global_position_int(const uint8_t *payload)
{
    int32_t  lat_1e7  = le_i32(payload + 4);
    int32_t  lon_1e7  = le_i32(payload + 8);
    int32_t  alt_mm   = le_i32(payload + 12);
    int16_t  vx       = le_i16(payload + 20);
    int16_t  vy       = le_i16(payload + 22);
    uint16_t hdg_cdeg = le_u16(payload + 26);

    float vx_ms = vx / 100.0f;
    float vy_ms = vy / 100.0f;
    float spd   = __builtin_sqrtf(vx_ms*vx_ms + vy_ms*vy_ms) * 3.6f;
    float hdg   = (hdg_cdeg != 65535) ? hdg_cdeg / 100.0f : -1.0f;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snap.lat       = lat_1e7 / 1e7;
    s_snap.lon       = lon_1e7 / 1e7;
    s_snap.alt_m     = alt_mm  / 1000.0f;
    s_snap.speed_kmph = spd;
    if (hdg >= 0.0f) {
        s_snap.heading_deg = hdg;
        s_snap.course_deg  = hdg;
    }
    xSemaphoreGive(s_mutex);
}

/* #74 VFR_HUD */
static void parse_vfr_hud(const uint8_t *payload)
{
    float   groundspeed = le_f32(payload + 4);
    int16_t heading     = le_i16(payload + 16);

    float hdg = (float)heading;
    if (hdg < 0.0f) hdg += 360.0f;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snap.speed_kmph  = groundspeed * 3.6f;
    s_snap.course_deg  = hdg;
    s_snap.heading_deg = hdg;
    xSemaphoreGive(s_mutex);
}

/* ═══════════════════════════════════════════════════════════
 * Frame dispatcher
 * ═══════════════════════════════════════════════════════════ */
static void dispatch_frame(uint8_t msg_id, const uint8_t *payload)
{
    switch (msg_id) {
    case MSG_SYSTEM_TIME:         parse_system_time(payload);         break;
    case MSG_GPS_RAW_INT:         parse_gps_raw_int(payload);         break;
    case MSG_GLOBAL_POSITION_INT: parse_global_position_int(payload); break;
    case MSG_VFR_HUD:             parse_vfr_hud(payload);             break;
    default: break;
    }
}

/* ═══════════════════════════════════════════════════════════
 * CRC extra byte lookup
 * ═══════════════════════════════════════════════════════════ */
static uint8_t get_crc_extra(uint8_t msg_id)
{
    for (int i = 0; i < (int)(sizeof(s_crc_table)/sizeof(s_crc_table[0])); i++) {
        if (s_crc_table[i].id == msg_id) return s_crc_table[i].crc_extra;
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════
 * MAVLink v1 parser state machine
 *
 * FIX: Parser state is now held in a persistent struct (mavlink_parser_t)
 * owned by the task, not in static local variables inside process_byte.
 * dispatch_frame() writes directly into s_snap under the mutex, so there
 * is no possibility of a parsed result being lost in a discarded copy.
 * ═══════════════════════════════════════════════════════════ */
typedef enum {
    PARSE_IDLE,
    PARSE_LEN,
    PARSE_SEQ,
    PARSE_SYS,
    PARSE_COMP,
    PARSE_MSGID,
    PARSE_PAYLOAD,
    PARSE_CRC1,
    PARSE_CRC2,
} parse_state_t;

typedef struct {
    parse_state_t state;
    uint8_t       buf[MAVLINK_MAX_FRAME];
    uint8_t       payload_len;
    uint8_t       payload_idx;
    uint8_t       msg_id;
    uint16_t      crc_rx;
    uint16_t      crc_calc;
} mavlink_parser_t;

static void process_byte(uint8_t byte, mavlink_parser_t *p)
{
    switch (p->state) {
    case PARSE_IDLE:
        if (byte == MAVLINK_STX) {
            p->buf[0] = byte;
            p->state  = PARSE_LEN;
        }
        break;

    case PARSE_LEN:
        p->payload_len = byte;
        p->buf[1]      = byte;
        p->crc_calc    = 0xFFFF;
        crc_accumulate(byte, &p->crc_calc);
        p->state = PARSE_SEQ;
        break;

    case PARSE_SEQ:
        p->buf[2] = byte;
        crc_accumulate(byte, &p->crc_calc);
        p->state = PARSE_SYS;
        break;

    case PARSE_SYS:
        p->buf[3] = byte;
        crc_accumulate(byte, &p->crc_calc);
        p->state = PARSE_COMP;
        break;

    case PARSE_COMP:
        p->buf[4] = byte;
        crc_accumulate(byte, &p->crc_calc);
        p->state = PARSE_MSGID;
        break;

    case PARSE_MSGID:
        p->msg_id  = byte;
        p->buf[5]  = byte;
        crc_accumulate(byte, &p->crc_calc);
        p->payload_idx = 0;
        p->state = (p->payload_len > 0) ? PARSE_PAYLOAD : PARSE_CRC1;
        break;

    case PARSE_PAYLOAD:
        p->buf[MAVLINK_HDR_LEN + p->payload_idx] = byte;
        crc_accumulate(byte, &p->crc_calc);
        if (++p->payload_idx >= p->payload_len) {
            crc_accumulate(get_crc_extra(p->msg_id), &p->crc_calc);
            p->state = PARSE_CRC1;
        }
        break;

    case PARSE_CRC1:
        p->crc_rx = byte;
        p->state  = PARSE_CRC2;
        break;

    case PARSE_CRC2:
        p->crc_rx |= ((uint16_t)byte << 8);
        if (p->crc_rx == p->crc_calc) {
            dispatch_frame(p->msg_id, p->buf + MAVLINK_HDR_LEN);
        }
        p->state = PARSE_IDLE;
        break;
    }
}

/* ═══════════════════════════════════════════════════════════
 * Background reader task
 * ═══════════════════════════════════════════════════════════ */
static void mavlink_reader_task(void *arg)
{
    mavlink_parser_t parser = {0};   /* persistent state for this task */
    uint8_t byte;

    for (;;) {
        int len = uart_read_bytes(MAV_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        /* process_byte updates s_snap directly under mutex when a
         * complete valid frame is received — no copy/discard risk. */
        process_byte(byte, &parser);
    }
}

/* ═══════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════ */
void mavlink_reader_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    uart_config_t uart_cfg = {
        .baud_rate  = MAV_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MAV_UART_PORT, MAV_UART_BUF_SIZE,
                                        0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART_PORT,
                                  MAV_UART_TX_PIN, MAV_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "MAVLink UART started (RX=GPIO%d %d baud)",
             MAV_UART_RX_PIN, MAV_UART_BAUD);

    xTaskCreate(mavlink_reader_task, "mav_reader", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "MAVLink reader task started");
}

void mavlink_reader_get(gps_snapshot_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_snap, sizeof(*out));
    xSemaphoreGive(s_mutex);
}