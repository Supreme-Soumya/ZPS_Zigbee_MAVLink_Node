#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "coordinator.h"
#include "zb_config_platform.h"

static const char *TAG = "COORDINATOR";

/* ── Commissioning callbacks ─────────────────────────────── */
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK,
        , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p           = signal_struct->p_app_signal;
    esp_err_t err_status        = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialising Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Starting network formation");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                esp_zb_bdb_open_network(180);
                ESP_LOGI(TAG, "Device rebooted — network open for 180 s");
            }
        } else {
            ESP_LOGE(TAG, "Zigbee stack init failed: %s",
                     esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee;
            esp_zb_get_long_address(ieee);
            ESP_LOGI(TAG, "Network formed PAN:0x%04hx CH:%d Addr:0x%04hx",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(),
                     esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started — waiting for devices");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params =
            (esp_zb_zdo_signal_device_annce_params_t *)
            esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Device joined: short=0x%04hx",
                 dev_annce_params->device_short_addr);
        break;

    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            uint8_t open = *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p);
            if (open)
                ESP_LOGI(TAG, "Network open for %d s", open);
            else
                ESP_LOGW(TAG, "Network closed");
        }
        break;

    default:
        break;
    }
}

/* ── GPS JSON parser ───────────────────────────────────────── */
static void parse_and_log_gps_json(const char *json)
{
    double lat = 0, lon = 0;
    float  alt = 0, spd = 0, crs = 0, hdg = 0, hdop = 0;
    int    sat = 0, fix = 0;
    char   utc[20] = {0};

    int matched = sscanf(json,
        "{"
        "\"a\":%lf,"
        "\"o\":%lf,"
        "\"e\":%f,"
        "\"s\":%f,"
        "\"c\":%f,"
        "\"h\":%f,"
        "\"n\":%d,"
        "\"d\":%f,"
        "\"f\":%d,"
        "\"t\":\"%19[^\"]\""
        "}",
        &lat, &lon, &alt, &spd, &crs, &hdg, &sat, &hdop, &fix, utc);

    if (matched < 9) {
        ESP_LOGW(TAG, "JSON parse incomplete (%d/10 fields): %s",
                 matched, json);
        return;
    }

    /* Pretty-print all fields  */
    ESP_LOGI(TAG, "┌─── GPS Update ──────────────────────────");
    ESP_LOGI(TAG, "│ Fix valid   : %s",          fix ? "YES" : "NO (waiting)");
    ESP_LOGI(TAG, "│ UTC         : %s",          utc[0] ? utc : "—");
    ESP_LOGI(TAG, "│ Lat / Lon   : %.6f, %.6f",  lat, lon);
    ESP_LOGI(TAG, "│ Altitude    : %.1f m",      alt);
    ESP_LOGI(TAG, "│ Speed       : %.1f km/h",   spd);
    ESP_LOGI(TAG, "│ Course      : %.1f °",      crs);
    ESP_LOGI(TAG, "│ Heading     : %.1f °",      hdg);
    ESP_LOGI(TAG, "│ Satellites  : %d",          sat);
    ESP_LOGI(TAG, "│ HDOP        : %.2f",        hdop);
    ESP_LOGI(TAG, "└─────────────────────────────────────────");
}

/* ── ZCL attribute callback ──────────────────────────────── */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                    const void *message)
{
    if (callback_id != ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) return ESP_OK;

    const esp_zb_zcl_set_attr_value_message_t *msg = message;
    if (msg->info.cluster != CUSTOM_CLUSTER_ID ||
        msg->attribute.id  != CUSTOM_ATTR_JSON_ID) {
        return ESP_OK;
    }

    /* ZCL character string: byte 0 = length, rest = payload */
    const uint8_t *zcl_str = (const uint8_t *)msg->attribute.data.value;
    uint8_t str_len = zcl_str[0];

    if (str_len == 0 || str_len >= CUSTOM_JSON_MAX_LEN) {
        ESP_LOGW(TAG, "Received ZCL string with unexpected length %d", str_len);
        return ESP_OK;
    }

    char json_buf[CUSTOM_JSON_MAX_LEN] = {0};
    memcpy(json_buf, zcl_str + 1, str_len);
    json_buf[str_len] = '\0';

    parse_and_log_gps_json(json_buf);
    return ESP_OK;
}

/* ── Zigbee stack task ────────────────────────────────────── */
static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    esp_zb_cluster_list_t  *cluster_list  = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluster,
        ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,  ESP_MODEL_IDENTIFIER);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list,
        esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

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
        .endpoint       = ESP_ZB_GATEWAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id  = ESP_ZB_HA_REMOTE_CONTROL_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_gateway_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

/* ── Entry point ─────────────────────────────────────────── */
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
