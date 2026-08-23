#pragma once
#include "esp_err.h"
#include "esp_zigbee_core.h"

/* ── Zigbee Configuration ────────────────────────────────── */
#define MAX_CHILDREN 10
#define INSTALLCODE_POLICY_ENABLE false
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
