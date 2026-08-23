#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ── UART to APM 2.8 TELEM port ──────────────────────────── */
#define MAV_UART_PORT     UART_NUM_1
#define MAV_UART_RX_PIN   6       /* APM TELEM TX → ESP32-C6 GPIO6 */
#define MAV_UART_TX_PIN   7       /* APM TELEM RX → ESP32-C6 GPIO7 */
#define MAV_UART_BAUD     57600   /* APM default TELEM baud rate    */
#define MAV_UART_BUF_SIZE 512

/*
 * GPS data snapshot — identical fields to gps_reader.h so that
 * end_device.c needs only an include swap and no other changes.
 */
typedef struct {
    double  lat;          /* degrees, +N / -S                */
    double  lon;          /* degrees, +E / -W                */
    float   alt_m;        /* altitude AMSL in metres         */
    float   speed_kmph;   /* ground speed (km/h)             */
    float   course_deg;   /* course over ground (degrees)    */
    float   heading_deg;  /* vehicle heading from FC         */
    float   hdop;         /* horizontal dilution of precision*/
    uint8_t satellites;   /* satellites visible              */
    uint8_t fix_valid;    /* 1 = 3-D fix or better           */
    char    utc[21];      /* "YYYY-MM-DDTHH:MM:SS"           */
} gps_snapshot_t;

/* ── Public API ───────────────────────────────────────────── */
/**
 * Initialises UART, then launches the background MAVLink reader task.
 * Call once from app_main before the Zigbee task.
 */
void mavlink_reader_init(void);

/**
 * Copy the latest GPS snapshot into *out.
 * Thread-safe (uses an internal mutex).
 */
void mavlink_reader_get(gps_snapshot_t *out);
