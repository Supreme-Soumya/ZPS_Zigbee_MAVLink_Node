/*
 * end_device.c — Zigbee End Device
 *
 * send_value_cb(): snprintf format uses compact single-character JSON
 * keys (a/o/e/s/c/h/n/d/f/t) instead of full words.
 *    
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_check.h"
#include "end_device.h"
#include "zb_config_platform.h"
#include "esp_event.h"
#include "mavlink_reader.h"

static void send_value_cb(uint8_t param);

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    esp_zb_bdb_start_top_level_commissioning(mode_mask);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p           = signal_struct->p_app_signal;
    esp_err_t err_status        = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI("ED", "Initialising Zigbee stack (End Device)");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI("ED", "FIRST_START/REBOOT signal, err=%s",
                 esp_err_to_name(err_status));
        if (err_status == ESP_OK) {
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGW("ED", "Stack init failed, retrying steering in 1s");
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI("ED",
                     "Joined network! Short addr=0x%04hx PAN=0x%04hx CH=%d",
                     esp_zb_get_short_address(), esp_zb_get_pan_id(),
                     esp_zb_get_current_channel());
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)send_value_cb, 0, 1000);
        } else {
            ESP_LOGW("ED", "Steering failed (err=%s), retrying in 5s",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;

    default:
        ESP_LOGI("ED", "Unhandled signal: 0x%lx err=%s",
                 (unsigned long)sig_type, esp_err_to_name(err_status));
        break;
    }
}

static void send_value_cb(uint8_t param)
{
    if (!esp_zb_bdb_dev_joined()) {
        ESP_LOGW("ED", "Not joined yet, will retry in 5s");
        esp_zb_scheduler_alarm((esp_zb_callback_t)send_value_cb, 0, 5000);
        return;
    }

    /* ── 1. Grab latest MAVLink data ──────────────────────── */
    gps_snapshot_t gps;
    mavlink_reader_get(&gps);

    /* ── 2. Build compact JSON into ZCL string buffer ────────
     *
     * Key mapping (must match coordinator.c parse_and_log_gps_json):
     *   a = lat       o = lon       e = alt (elevation)
     *   s = speed     c = course    h = heading
     *   n = satellites (number)     d = hdop (dilution)
     *   f = fix       t = time (UTC)
     * ─────────────────────────────────────────────────────── */
    uint8_t zcl_str[CUSTOM_JSON_MAX_LEN] = {0};
    char   *json_body = (char *)(zcl_str + 1);

    int json_len = snprintf(json_body, CUSTOM_JSON_MAX_LEN - 1,
        "{"
        "\"a\":%.6f,"
        "\"o\":%.6f,"
        "\"e\":%.1f,"
        "\"s\":%.1f,"
        "\"c\":%.1f,"
        "\"h\":%.1f,"
        "\"n\":%u,"
        "\"d\":%.2f,"
        "\"f\":%u,"
        "\"t\":\"%s\""
        "}",
        gps.lat, gps.lon, gps.alt_m,
        gps.speed_kmph, gps.course_deg, gps.heading_deg,
        gps.satellites, gps.hdop, gps.fix_valid,
        gps.utc[0] ? gps.utc : "");

    zcl_str[0] = (uint8_t)(json_len & 0xFF);

    ESP_LOGI("ED", "Sending GPS JSON (%d bytes): %s", json_len, json_body);

    /* ── 3. ZCL write-attribute command ───────────────────── */
    esp_zb_zcl_write_attr_cmd_t write_cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000,
            .dst_endpoint          = 1,
            .src_endpoint          = ESP_ZB_GATEWAY_ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID    = CUSTOM_CLUSTER_ID,
        .attr_number  = 1,
        .attr_field   = (esp_zb_zcl_attribute_t[]) {{
            .id   = CUSTOM_ATTR_JSON_ID,
            .data = {
                .type  = ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
                .value = zcl_str,
            },
        }},
    };
    esp_zb_zcl_write_attr_cmd_req(&write_cmd);

    esp_zb_scheduler_alarm((esp_zb_callback_t)send_value_cb, 0, 1000);
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    esp_zb_cluster_list_t  *cluster_list   = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(NULL),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    uint8_t init_str[CUSTOM_JSON_MAX_LEN] = {0};
    esp_zb_attribute_list_t *custom_cluster =
        esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster,
        CUSTOM_ATTR_JSON_ID,
        ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
        init_str);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint           = ESP_ZB_GATEWAY_ENDPOINT,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);

    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_scheduler_alarm((esp_zb_callback_t)send_value_cb, 0, 1000);

    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Only erase NVS if corrupted or version mismatch */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    mavlink_reader_init();
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
