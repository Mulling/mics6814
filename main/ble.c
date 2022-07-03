// Copyright (C) 2022 Lucas Mulling

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

static const char *TAG = "ble";

#include "ble.h"

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_err.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gatts_api.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <string.h>

#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_WARN

#include "utils.h"

#define DEVICE_NAME "MICS-6814"

#define ESP_APP_ID  0x55
#define LOCAL_MTU   0x1F4
#define SVC_INST_ID 0x0

#define CHAR_VAL_LEN_MAX (CHAR_VAL_NUM * sizeof(uint8_t))
#define CHAR_VAL_NUM     (3)

#define ES_CONNECTED_FMT "ES CONNECTED=%u"
#define ES_NOTIFY_FMT    "ES NOTIFY=%u"

enum {
    IDX_SVC = 0x00,
    IDX_CHAR_NH3,
    IDX_CHAR_NH3_VAL,
    IDX_CHAR_NH3_CFG,
    IDX_SIZE
};

typedef struct ble_conn {
    uint16_t conn_id;
    esp_gatt_if_t gatts_if;
    struct ble_conn *next;
} ble_conn;

ble_conn *ble_conn_lst;

void ble_conn_lst_insert(const esp_gatt_if_t, const uint16_t);
void ble_conn_lst_remove(const uint16_t);
void ble_conn_lst_map(void (*)(const esp_gatt_if_t, const uint16_t, uint8_t*, const uint16_t), uint8_t*, const uint16_t);

static uint16_t nh3_handle_table[IDX_SIZE];

static const uint16_t svc_uuid                         = 0x181A; // environmental sensing
static const uint16_t char_uuid                        = 0xB2CF; // ammonia concentration
static const uint16_t esp_gatt_uuid_char_client_config = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t esp_gatt_uuid_char_declare       = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t esp_gatt_uuid_pri_service        = ESP_GATT_UUID_PRI_SERVICE;

static const uint8_t esp_gatt_char_prop_bit_rn = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t nh3_value[CHAR_VAL_NUM] = {0x30, 0x30, 0x30};
static const uint8_t nh3_ccc[2]              = {0x00, 0x00};

static uint8_t conn = 0; // number of active connections, should be less than 4

static QueueHandle_t ble_adv_msg_queue;

// GATT service database
static const esp_gatts_attr_db_t gatt_db[IDX_SIZE] = {
    [IDX_SVC] = {{
        .auto_rsp = ESP_GATT_AUTO_RSP
    },
    {
        .uuid_length = ESP_UUID_LEN_16,
        .uuid_p      = (uint8_t *)&esp_gatt_uuid_pri_service,
        .perm        = ESP_GATT_PERM_READ,
        .max_length  = sizeof(uint16_t),
        .length      = sizeof(svc_uuid),
        .value       = (uint8_t *)&svc_uuid
    }
    },
    [IDX_CHAR_NH3] = {{
        .auto_rsp = ESP_GATT_AUTO_RSP
    },
    {
        .uuid_length = ESP_UUID_LEN_16,
        .uuid_p      = (uint8_t *)&esp_gatt_uuid_char_declare,
        .perm        = ESP_GATT_PERM_READ,
        .max_length  = sizeof(uint8_t),
        .length      = sizeof(esp_gatt_uuid_char_declare),
        .value       = (uint8_t *)&esp_gatt_char_prop_bit_rn
    }
    },
    [IDX_CHAR_NH3_VAL] = {{
        .auto_rsp = ESP_GATT_AUTO_RSP
    },
    {
        .uuid_length = ESP_UUID_LEN_16,
        .uuid_p      = (uint8_t *)&char_uuid,
        .perm        = ESP_GATT_PERM_READ,
        .max_length  = CHAR_VAL_LEN_MAX,
        .length      = sizeof(nh3_value),
        .value       = (uint8_t *)&nh3_value
    }
    },
    [IDX_CHAR_NH3_CFG] = {{
        .auto_rsp = ESP_GATT_AUTO_RSP
    },
    {
        .uuid_length = ESP_UUID_LEN_16,
        .uuid_p      = (uint8_t *)&esp_gatt_uuid_char_client_config,
        .perm        = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        .max_length  = sizeof(uint16_t),
        .length      = sizeof(nh3_ccc),
        .value       = (uint8_t *)&nh3_ccc
    }
    }
};

static uint8_t raw_adv_data[31] = {
    0x02, 0x0A, 0xEB,       // tx power
    0x03, 0x03, 0x2B, 0xCF, // 16-bit service uuid
    0x03, 0x19, 0x05, 0x42, // appearance values
    0x0A, 0x09, 'M', 'I', 'C', 'S', '-', '6', '8', '1', '4'
}; //            ^
   //            |
   //            |
   //            +---------------+
   //                            |
#define ADV_DATA_LOCAL_NAME     13
#define ADV_DATA_LOCAL_NAME_LEN 11

static uint8_t raw_scan_rsp_data[31] = {
    0x02, 0x0A, 0xEB,       // tx power
    0x03, 0x03, 0x2B, 0xCF, // 16-bit service uuid
    0x03, 0x19, 0x05, 0x42, // appearance value
    0x03, 0x16, 0x00, 0x00  // service data
}; //            ^
   //            |
   //            |
   //            +-------------+
   //                          |
#define SCAN_RSP_SERVICE_DATA 13

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

void ble_notify_cb(
        const esp_gatt_if_t gatts_if,
        const uint16_t conn_id,
        uint8_t *value,
        const uint16_t size){
    ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(gatts_if, conn_id, nh3_handle_table[IDX_CHAR_NH3_VAL], size, value, false));
}

inline __attribute__((always_inline))
void ble_set_nh3_attr(uint8_t *value, const uint16_t size){
    // NONE: this will trigger ESP_GATTS_SET_ATTR_VAL_EVT which will notify all the notifiable connections
    ESP_ERROR_CHECK(esp_ble_gatts_set_attr_value(nh3_handle_table[IDX_CHAR_NH3_VAL], size, (uint8_t *)value));
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    switch(event){
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGD(TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGD(TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGD(TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
            {
                uint8_t ble_adv_msg[20 + 1];
                uint8_t n;

                if (xQueueReceive(ble_adv_msg_queue, (void *)ble_adv_msg, (TickType_t)0) == pdPASS){

                    memcpy(&raw_adv_data[ADV_DATA_LOCAL_NAME], ble_adv_msg + 2, (n = strlen((char *)ble_adv_msg + 2)));
                    // including 0x09
                    raw_adv_data[ADV_DATA_LOCAL_NAME_LEN] = n + 1;
                    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));

                    memcpy(&raw_scan_rsp_data[SCAN_RSP_SERVICE_DATA], ble_adv_msg, sizeof(uint16_t));
                    ESP_ERROR_CHECK(esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data)));
                } else {
                    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
                }
            }
            break;

        default:
            ESP_LOGD(TAG, "GAP unhandled event %d", event);
            break;
    }
}

void ble_update_adv_msg(const uint16_t service_data, const char *restrict fmt, ...){
    // NOTE: maximum msg size is 18 + \0
    char adv_msg[20 + 1];
    va_list args;

    va_start(args, fmt);
    vsnprintf(adv_msg + 2, (size_t)(18 + 1), fmt, args);
    va_end(args);

    memcpy(adv_msg, &service_data, sizeof(uint16_t));

    xQueueSend(ble_adv_msg_queue, (void*)adv_msg, (TickType_t)0);
    ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
}

void gatts_event_handler(
        esp_gatts_cb_event_t event,
        esp_gatt_if_t gatts_if,
        esp_ble_gatts_cb_param_t *param){
    switch (event){
        case ESP_GATTS_REG_EVT:
            ESP_LOGD(TAG, "ESP_GATTS_REG_EVT");

            ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));
            ESP_ERROR_CHECK(esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data)));
            ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_SIZE, SVC_INST_ID));
            break;

        case ESP_GATTS_WRITE_EVT:
            switch (param->write.value[0]){
                case 0x00:
                    ESP_LOGD(TAG, "ESP_GATTS_WRITE_EVT notify/indicate disable");
                    ble_conn_lst_remove(param->write.conn_id);
                    break;

                case 0x01:
                    ESP_LOGD(TAG, "ESP_GATTS_WRITE_EVT notify enable");
                    ble_conn_lst_insert(gatts_if, param->write.conn_id);
                    break;

                default:
                    break;
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            {
                ESP_LOGD(TAG, "ESP_GATTS_CONNECT_EVT conn_id = %d", param->connect.conn_id);
                // esp_log_buffer_hex(TAG, param->connect.remote_bda, 5);

                esp_ble_conn_update_params_t conn_params = {
                    .latency = 0,
                    .max_int = 0x20, // 0x20 * 1.25ms = 40ms
                    .min_int = 0x10, // 0x10 * 1.25ms = 20ms
                    .timeout = 1000, // 1000 * 10ms   = 10000ms
                };
                memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

                esp_ble_gap_update_conn_params(&conn_params);
                oprintf(4, ES_CONNECTED_FMT, ++conn);
            }
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGD(TAG, "ESP_GATTS_DISCONNECT_EVT reason = 0x%x", param->disconnect.reason);
            // if the connection is in the notify list, remove it
            ble_conn_lst_remove(param->disconnect.conn_id);
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
            if (!(--conn))
                oprintf(4, "");
            else
                oprintf(4, ES_CONNECTED_FMT, conn);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK && param->add_attr_tab.num_handle == IDX_SIZE){
                ESP_LOGD(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT handle = %d", param->add_attr_tab.num_handle);

                memcpy(nh3_handle_table, param->add_attr_tab.handles, sizeof(nh3_handle_table));
                esp_ble_gatts_start_service(nh3_handle_table[IDX_SVC]);
            }
            break;

        case ESP_GATTS_SET_ATTR_VAL_EVT:
            ESP_LOGD(TAG, "ESP_GATTS_SET_ATTR_VAL_EVT");

            const uint8_t *value_ptr;
            uint16_t lentgh;

            ESP_ERROR_CHECK(esp_ble_gatts_get_attr_value(param->set_attr_val.attr_handle, &lentgh, &value_ptr));

            uint8_t value[CHAR_VAL_NUM];
            memcpy(value, value_ptr, sizeof(value));

            ble_conn_lst_map(ble_notify_cb, value, sizeof(value));
            break;

        default:
            ESP_LOGD(TAG, "GATTS unhandled event %d", event);
            break;
    }
}

inline __attribute__((always_inline))
esp_err_t ble_init(){
    ble_conn_lst = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t esp_bt_controller_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&esp_bt_controller_config));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(ESP_APP_ID));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(LOCAL_MTU));

    // NOTE: first two bytes are the service data, the remaining bytes are the adv msg including \0
    if((ble_adv_msg_queue = xQueueCreate(10, sizeof(char) * (20 + 1))) == NULL)
        ESP_LOGE(TAG, "fail to crate ble_msg_queue");

    return ESP_OK;
}

inline __attribute__((always_inline))
void ble_conn_lst_insert(const esp_gatt_if_t gatts_if, const uint16_t conn_id){
    ble_conn **head = &ble_conn_lst;

    uint8_t size = 0;

    while ((*head)){
        // NOTE: this should not happen since the maximum number of connections is controlled by the BT controller
        if ((++size) == CONFIG_BTDM_CTRL_BLE_MAX_CONN)
            return;
        head = &(*head)->next;
    }

    ble_conn *new = malloc(sizeof(ble_conn));

    new->conn_id = conn_id;
    new->gatts_if = gatts_if;
    new->next = NULL;

    *head = new;

    oprintf(5, ES_NOTIFY_FMT, size + 1);
}

inline __attribute__((always_inline))
void ble_conn_lst_remove(const uint16_t conn_id){
    ble_conn **head = &ble_conn_lst;

    while ((*head)){
        if ((*head)->conn_id == conn_id){
            ble_conn *old = *head;
            *head = (*head)->next;
            free(old);
            if ((*head) == NULL) oprintf(5, "");
            return;
        }

        head = &(*head)->next;
    }
}

inline __attribute__((always_inline))
void ble_conn_lst_map(
        void (*fn)(const esp_gatt_if_t, const uint16_t, uint8_t*, const uint16_t),
        uint8_t *value,
        const uint16_t size){
    ble_conn **head = &ble_conn_lst;

    while ((*head)){
        fn((*head)->gatts_if, (*head)->conn_id, value, size);
        head = &(*head)->next;
    }
}
