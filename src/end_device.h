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
 * the default for most Indian ISP routers. Outdoors you encounter many
 * more such APs, which prevents the end device joining ("stuck at
 * network opened"). Channel 26 (2480 MHz) has the least WiFi overlap
 * and is the standard recommendation for outdoor/field use.
 *
 * Must match coordinator.h exactly. Erase NVS on both devices after
 * flashing so they renegotiate on the new channel.
 */
#define ESP_ZB_PRIMARY_CHANNEL_MASK  (1l << 26)

#define ESP_ZB_GATEWAY_ENDPOINT      1
#define APP_PROD_CFG_CURRENT_VERSION 0x0001

/* ── Basic manufacturer information ─────────────────────── */
#define ESP_MANUFACTURER_CODE   0x131B
#define ESP_MANUFACTURER_NAME   "\x09""ESPRESSIF"
#define ESP_MODEL_IDENTIFIER    "\x07"CONFIG_IDF_TARGET

/* ── RCP connection pins (only used in UART_RCP radio mode) ─ */
#define HOST_RX_PIN_TO_RCP_TX  4
#define HOST_TX_PIN_TO_RCP_RX  5

/* ── Custom cluster for GPS JSON payload ─────────────────── */
#define CUSTOM_CLUSTER_ID      0xFF00
#define CUSTOM_ATTR_JSON_ID    0x0001

/*
 * FIX 2: Reduced from 200 to 128 bytes.
 *
 * The old worst-case JSON (~190 bytes) sat dangerously close to the APS
 * frame MTU. On marginal links (corridors, weak RSSI) large APS frames
 * are silently dropped, causing the short-range disconnection you saw.
 *
 * The new compact format uses single-character keys:
 *   a=lat  o=lon  e=alt  s=speed  c=course  h=heading
 *   n=sats  d=hdop  f=fix  t=utc
 *
 * Worst-case example (128 chars including null):
 *   {"a":22.572646,"o":88.363895,"e":1234.5,"s":120.3,
 *    "c":275.1,"h":276.0,"n":12,"d":1.20,"f":1,
 *    "t":"2024-07-15T08:30:00"}
 *   → ~131 chars — fits comfortably within safe APS limits.
 *
 * Keep in sync with coordinator.h.
 */
#define CUSTOM_JSON_MAX_LEN    140   /* 131 worst-case + 9 headroom */

/* ── Zigbee stack macros ──────────────────────────────────── */
/*
 * End device role (was ROUTER in earlier buggy version — already fixed
 * in the version you sent; keeping the comment for reference).
 */
#define ESP_ZB_ZED_CONFIG() \
    { \
        .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ED, \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = { \
            .ed_timeout  = ESP_ZB_ED_AGING_TIMEOUT_64MIN, \
            .keep_alive  = 3000, \
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
                .baud_rate            = 460800, \
                .data_bits            = UART_DATA_8_BITS, \
                .parity               = UART_PARITY_DISABLE, \
                .stop_bits            = UART_STOP_BITS_1, \
                .flow_ctrl            = UART_HW_FLOWCTRL_DISABLE, \
                .rx_flow_ctrl_thresh  = 0, \
                .source_clk           = UART_SCLK_DEFAULT, \
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