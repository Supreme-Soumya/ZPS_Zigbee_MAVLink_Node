#pragma once
#include "esp_err.h"
#include "esp_zigbee_core.h"

/* ── Zigbee Configuration ────────────────────────────────── */
#define MAX_CHILDREN 10
#define INSTALLCODE_POLICY_ENABLE false

/*
 * FIX 1: Changed from channel 13 (1l << 13) to channel 26 (1l << 26).
 *
 * Zigbee channel 13 (2471 MHz) overlaps heavily with WiFi channel 11,
 * which is the default for most Indian ISP routers (BSNL, Airtel, etc.).
 * Outdoors you encounter far more of these APs, causing interference that
 * prevents the end device from joining ("stuck at network opened").
 *
 * Zigbee channel 26 (2480 MHz) has the least WiFi overlap of all Zigbee
 * channels and is the standard recommendation for outdoor/field deployments.
 *
 * IMPORTANT: Flash both coordinator and end device with this change,
 * then erase NVS on both before powering on, so they renegotiate on
 * the new channel:
 *   idf.py -p <PORT> erase-flash   (or just erase the NVS partition)
 */
#define ESP_ZB_PRIMARY_CHANNEL_MASK (1l << 26)

#define ESP_ZB_GATEWAY_ENDPOINT     1
#define APP_PROD_CFG_CURRENT_VERSION 0x0001

/* ── Basic manufacturer information ─────────────────────── */
#define ESP_MANUFACTURER_CODE    0x131B
#define ESP_MANUFACTURER_NAME    "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER     "\x07"CONFIG_IDF_TARGET

/* ── RCP connection pins (only used in UART_RCP radio mode) ─ */
#define HOST_RX_PIN_TO_RCP_TX   4
#define HOST_TX_PIN_TO_RCP_RX   5

/* ── Custom cluster — must match end_device.h exactly ───── */
#define CUSTOM_CLUSTER_ID        0xFF00
#define CUSTOM_ATTR_JSON_ID      0x0001

/*
 * FIX 2: Reduced from 200 to 128 bytes.
 *
 * The previous worst-case JSON (~190 bytes) was dangerously close to the
 * APS frame MTU. On marginal links (corridors, weak RSSI) large frames
 * are dropped silently, causing intermittent disconnection at short range.
 *
 * The new compact JSON format (short keys a/o/e/s/c/h/n/d/f/t) produces
 * a worst-case payload of ~130 bytes, well within safe APS frame limits.
 *
 * Keep this in sync with end_device.h.
 */
#define CUSTOM_JSON_MAX_LEN      128

/* ── Disable WiFi for this project ──────────────────────── */
#undef  CONFIG_EXAMPLE_CONNECT_WIFI
#define CONFIG_EXAMPLE_CONNECT_WIFI 0

/* ── Zigbee stack macros ──────────────────────────────────── */
#define ESP_ZB_ZC_CONFIG() \
    { \
        .esp_zb_role            = ESP_ZB_DEVICE_TYPE_COORDINATOR, \
        .install_code_policy    = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zczr_cfg = { \
            .max_children = MAX_CHILDREN, \
        }, \
    }

#if CONFIG_ZB_RADIO_NATIVE
#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
    { \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }
#else
#define ESP_ZB_DEFAULT_RADIO_CONFIG() \
    { \
        .radio_mode = ZB_RADIO_MODE_UART_RCP, \
        .radio_uart_config = { \
            .port = 1, \
            .uart_config = { \
                .baud_rate         = 460800, \
                .data_bits         = UART_DATA_8_BITS, \
                .parity            = UART_PARITY_DISABLE, \
                .stop_bits         = UART_STOP_BITS_1, \
                .flow_ctrl         = UART_HW_FLOWCTRL_DISABLE, \
                .rx_flow_ctrl_thresh = 0, \
                .source_clk        = UART_SCLK_DEFAULT, \
            }, \
            .rx_pin = HOST_RX_PIN_TO_RCP_TX, \
            .tx_pin = HOST_TX_PIN_TO_RCP_RX, \
        }, \
    }
#endif

#define ESP_ZB_DEFAULT_HOST_CONFIG() \
    { \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }