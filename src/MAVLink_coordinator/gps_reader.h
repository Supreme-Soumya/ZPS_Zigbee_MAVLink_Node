#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ── GPS UART (UART1) ─────────────────────────────────────── */
#define GPS_UART_PORT       UART_NUM_1
#define GPS_UART_RX_PIN     6       /* NEO-7M TX  → ESP32-C6 GPIO6  */
#define GPS_UART_TX_PIN     7       /* NEO-7M RX  → ESP32-C6 GPIO7  */
#define GPS_UART_BAUD       9600
#define GPS_UART_BUF_SIZE   512

/* ── Compass I2C (HMC5883L) ───────────────────────────────── */
#define COMPASS_I2C_PORT    I2C_NUM_0
#define COMPASS_SDA_PIN     10       /* HMC5883L SDA → ESP32-C6 GPIO10 */
#define COMPASS_SCL_PIN     11       /* HMC5883L SCL → ESP32-C6 GPIO11 */
#define COMPASS_I2C_FREQ    100000
#define HMC5883L_ADDR       0x1E

/* ── GPS data snapshot ────────────────────────────────────── */
typedef struct {
    double   lat;           /* degrees, +N / -S           */
    double   lon;           /* degrees, +E / -W           */
    float    alt_m;         /* altitude in metres          */
    float    speed_kmph;    /* speed over ground (km/h)   */
    float    course_deg;    /* true course from GPRMC     */
    float    heading_deg;   /* magnetic heading (compass) */
    float    hdop;          /* horizontal dilution        */
    uint8_t  satellites;    /* satellites in use          */
    uint8_t  fix_valid;     /* 1 = valid GPRMC fix        */
    char     utc[20];       /* "YYYY-MM-DDTHH:MM:SS"      */
} gps_snapshot_t;

/* ── Public API ───────────────────────────────────────────── */

/**
 * Initialises GPS UART and compass I2C, then launches the
 * background reader task.  Call once from app_main / esp_zb_task.
 */
void gps_reader_init(void);

/**
 * Copy the latest GPS snapshot into *out.
 * Thread-safe (uses an internal mutex).
 */
void gps_reader_get(gps_snapshot_t *out);
