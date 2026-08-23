#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "gps_reader.h"

static const char *TAG = "GPS_READER";

/* ── Internal state ───────────────────────────────────────── */
static gps_snapshot_t    s_snap  = {0};
static SemaphoreHandle_t s_mutex = NULL;

/* ═══════════════════════════════════════════════════════════
 *  HMC5883L helpers
 * ═══════════════════════════════════════════════════════════ */
#define HMC_REG_CFG_A   0x00
#define HMC_REG_CFG_B   0x01
#define HMC_REG_MODE    0x02
#define HMC_REG_DATA    0x03    /* 6 bytes: Xh Xl Zh Zl Yh Yl */

static esp_err_t hmc_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(COMPASS_I2C_PORT, HMC5883L_ADDR,
                                      buf, 2, pdMS_TO_TICKS(20));
}

static esp_err_t hmc_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(COMPASS_I2C_PORT, HMC5883L_ADDR,
                                        &reg, 1, data, len,
                                        pdMS_TO_TICKS(20));
}

static void compass_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = COMPASS_SDA_PIN,
        .scl_io_num       = COMPASS_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = COMPASS_I2C_FREQ,
    };

    if (i2c_param_config(COMPASS_I2C_PORT, &cfg) != ESP_OK) {
        ESP_LOGW(TAG, "Compass I2C config failed — continuing without compass");
        return;
    }
    if (i2c_driver_install(COMPASS_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        ESP_LOGW(TAG, "Compass I2C install failed — continuing without compass");
        return;
    }

    /* 8-sample average, 15 Hz, normal measurement */
    hmc_write(HMC_REG_CFG_A, 0x70);
    /* Gain = 1090 LSb/Gauss (default) */
    hmc_write(HMC_REG_CFG_B, 0x20);
    /* Continuous-measurement mode */
    hmc_write(HMC_REG_MODE,  0x00);

    ESP_LOGI(TAG, "HMC5883L initialised");
}

static float compass_read_heading(void)
{
    uint8_t raw[6];
    if (hmc_read(HMC_REG_DATA, raw, 6) != ESP_OK) {
        return -1.0f;
    }

    /* Register order: Xh Xl  Zh Zl  Yh Yl  (note Z before Y!) */
    int16_t mx = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t my = (int16_t)((raw[4] << 8) | raw[5]);

    float heading = atan2f((float)my, (float)mx) * (180.0f / (float)M_PI);
    if (heading < 0.0f)   heading += 360.0f;
    if (heading > 360.0f) heading -= 360.0f;
    return heading;
}

/* ═══════════════════════════════════════════════════════════
 *  NMEA helpers
 * ═══════════════════════════════════════════════════════════ */

static int nmea_split(char *sentence, char *fields[], int max_fields)
{
    int n = 0;
    char *p = sentence;
    while (*p && n < max_fields) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (!p) break;
        *p++ = '\0';
    }
    return n;
}

static double nmea_to_deg(const char *val, char hemi)
{
    if (!val || *val == '\0') return 0.0;
    double raw = atof(val);
    double deg = (int)(raw / 100.0);
    double min = raw - deg * 100.0;
    double result = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') result = -result;
    return result;
}

static void parse_gpgga(char *fields[], int nf, gps_snapshot_t *s)
{
    if (nf < 10) return;
    if (fields[6][0] == '0') return;

    s->lat        = nmea_to_deg(fields[2], fields[3][0]);
    s->lon        = nmea_to_deg(fields[4], fields[5][0]);
    s->satellites = (uint8_t)atoi(fields[7]);
    s->hdop       = atof(fields[8]);
    s->alt_m      = atof(fields[9]);

    const char *t = fields[1];
    if (strlen(t) >= 6) {
        char hh[3] = {t[0], t[1], 0};
        char mm[3] = {t[2], t[3], 0};
        char ss[3] = {t[4], t[5], 0};
        if (s->utc[0] != '\0') {
            s->utc[11] = hh[0]; s->utc[12] = hh[1];
            s->utc[14] = mm[0]; s->utc[15] = mm[1];
            s->utc[17] = ss[0]; s->utc[18] = ss[1];
        } else {
            snprintf(s->utc, sizeof(s->utc),
                     "0000-00-00T%s:%s:%s", hh, mm, ss);
        }
    }
}

static void parse_gprmc(char *fields[], int nf, gps_snapshot_t *s)
{
    if (nf < 10) return;

    s->fix_valid = (fields[2][0] == 'A') ? 1 : 0;
    if (!s->fix_valid) return;

    s->lat        = nmea_to_deg(fields[3], fields[4][0]);
    s->lon        = nmea_to_deg(fields[5], fields[6][0]);
    s->speed_kmph = (float)(atof(fields[7]) * 1.852);
    s->course_deg = atof(fields[8]);

    const char *d = fields[9];
    const char *t = fields[1];
    if (strlen(d) >= 6 && strlen(t) >= 6) {
        snprintf(s->utc, sizeof(s->utc),
                 "20%c%c-%c%c-%c%cT%c%c:%c%c:%c%c",
                 d[4], d[5], d[2], d[3], d[0], d[1],
                 t[0], t[1], t[2], t[3], t[4], t[5]);
    }
}

static void nmea_process_line(char *line, gps_snapshot_t *s)
{
    char *star = strchr(line, '*');
    if (star) *star = '\0';

    char *fields[20];
    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    int nf = nmea_split(buf, fields, 20);
    if (nf < 2) return;

    const char *id = fields[0];
    if (strcmp(id, "$GPGGA") == 0 || strcmp(id, "$GNGGA") == 0) {
        parse_gpgga(fields, nf, s);
    } else if (strcmp(id, "$GPRMC") == 0 || strcmp(id, "$GNRMC") == 0) {
        parse_gprmc(fields, nf, s);
    }
}

/* ═══════════════════════════════════════════════════════════
 *  Background reader task
 * ═══════════════════════════════════════════════════════════ */
static void gps_reader_task(void *arg)
{
    uint32_t byte_count = 0;
    char     line_buf[128];
    int      line_pos = 0;
    uint8_t  byte;

    for (;;) {
        int len = uart_read_bytes(GPS_UART_PORT, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        byte_count++;
        if (byte_count % 100 == 0) {
            ESP_LOGI("GPS_RAW", "Bytes received so far: %lu", byte_count);
        }

        if (byte == '\r') continue;

        if (byte == '\n') {
            line_buf[line_pos] = '\0';
            line_pos = 0;

            if (line_buf[0] == '$') {
                gps_snapshot_t tmp;

                xSemaphoreTake(s_mutex, portMAX_DELAY);
                memcpy(&tmp, &s_snap, sizeof(tmp));
                xSemaphoreGive(s_mutex);

                nmea_process_line(line_buf, &tmp);

                /* Read compass only once per second */
                static uint32_t last_compass_ms = 0;
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (now - last_compass_ms >= 1000) {
                    float hdg = compass_read_heading();
                    if (hdg >= 0.0f) tmp.heading_deg = hdg;
                    last_compass_ms = now;
                }

                xSemaphoreTake(s_mutex, portMAX_DELAY);
                memcpy(&s_snap, &tmp, sizeof(s_snap));
                xSemaphoreGive(s_mutex);
            }
            continue;
        }

        if (line_pos < (int)sizeof(line_buf) - 1) {
            line_buf[line_pos++] = (char)byte;
        }
    }
}

/* ═══════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════ */
void gps_reader_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    uart_config_t uart_cfg = {
        .baud_rate  = GPS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT, GPS_UART_BUF_SIZE,
                                        0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT,
                                 GPS_UART_TX_PIN, GPS_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "GPS UART started (RX=GPIO%d, TX=GPIO%d, %d baud)",
             GPS_UART_RX_PIN, GPS_UART_TX_PIN, GPS_UART_BAUD);

    compass_init();

    xTaskCreate(gps_reader_task, "gps_reader", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "GPS reader task started");
}

void gps_reader_get(gps_snapshot_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_snap, sizeof(*out));
    xSemaphoreGive(s_mutex);
}