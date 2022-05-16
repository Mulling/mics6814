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

// FIXME: seems to not work sometimes...
#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_WARN

#include "ble_conn_lst.h"
#include "utils.h"

#define DEVICE_NAME "MICS-6814"

#define ESP_APP_ID  0x55
#define LOCAL_MTU   0x1F4
#define SVC_INST_ID 0x0

#define CHAR_VAL_LEN_MAX (CHAR_VAL_NUM * sizeof(uint8_t))
#define CHAR_VAL_NUM     (4)

enum {
    IDX_SVC = 0x00,
    IDX_CHAR_NH3,
    IDX_CHAR_NH3_VAL,
    IDX_CHAR_NH3_CFG,
    IDX_SIZE
};

ble_conn *ble_conn_lst;

static uint16_t nh3_handle_table[IDX_SIZE];

static const uint16_t svc_uuid                         = 0x181A; // environmental sensing
static const uint16_t char_uuid                        = 0xB2CF; // ammonia concentration
static const uint16_t esp_gatt_uuid_char_client_config = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t esp_gatt_uuid_char_declare       = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t esp_gatt_uuid_pri_service        = ESP_GATT_UUID_PRI_SERVICE;

static const uint8_t esp_gatt_char_prop_bit_rn = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t nh3_value[CHAR_VAL_NUM] = {0x30, 0x30, 0x30, 0x30};
static const uint8_t nh3_ccc[2]              = {0x00, 0x00};

static uint8_t conn = 0; // number of active connection, should be less than 4

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
    0x02, 0x01, 0x06,       // flags
    0x02, 0x0A, 0xEB,       // tx power
    0x03, 0x03, 0x2B, 0xCF, // 16-bit service uuid
    0x03, 0x19, 0x05, 0x42, // appearance values
    0x0A, 0x09, 'M', 'I', 'C', 'S', '-', '6', '8', '1', '4'
};

static uint8_t raw_scan_rsp_data[31] = {
    // TODO: add service data https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Generic%20Access%20Profile.pdf
    0x02, 0x01, 0x06,       // flags
    0x02, 0x0A, 0xEB,       // tx power
    0x03, 0x03, 0x2B, 0xCF, // 16-bit service uuid
    0x03, 0x19, 0x05, 0x42, // appearance values
};

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
            ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT");
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
            {
                char ble_adv_msg[18 + 1];
                size_t n;

                if (xQueueReceive(ble_adv_msg_queue, (void*)ble_adv_msg, (TickType_t)0) == pdPASS){

                    memcpy(&raw_adv_data[16], ble_adv_msg, (n = strlen(ble_adv_msg)));
                    raw_adv_data[14] = n + 1;

                    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));
                }
                else{
                    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
                }
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                    param->update_conn_params.status,
                    param->update_conn_params.min_int,
                    param->update_conn_params.max_int,
                    param->update_conn_params.conn_int,
                    param->update_conn_params.latency,
                    param->update_conn_params.timeout);
            break;
        default:
            ESP_LOGI(TAG, "GAP unhandled event %d", event);
            break;
    }
}

void ble_update_adv_msg(const char *restrict fmt, ...){
    char adv_msg[18 + 1];
    va_list args;

    va_start(args, fmt);
    vsnprintf(adv_msg, (size_t)(18 + 1), fmt, args);
    va_end(args);

    xQueueSend(ble_adv_msg_queue, (void*)adv_msg, (TickType_t)0);
    ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
}

void gatts_event_handler(esp_gatts_cb_event_t event,
        esp_gatt_if_t gatts_if,
        esp_ble_gatts_cb_param_t *param){
    switch (event){
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_REG_EVT");

            ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));
            ESP_ERROR_CHECK(esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data)));
            ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_SIZE, SVC_INST_ID));
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT");
            switch (param->write.value[0]){
                case 0x00:
                    ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT notify/indicate disable");
                    ble_conn_lst_remove(param->write.conn_id);
                    break;

                case 0x01:
                    ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT notify enable");
                    ble_conn_lst_insert(gatts_if, param->write.conn_id);
                    break;

                case 0x02:
                    ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT indicate enable"); // NOTE: not supported
                    break;

                default:
                    break;
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT MTU %d",
                    param->mtu.mtu);
            break;

        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT status = %d, attr_handle %d",
                    param->conf.status,
                    param->conf.handle);
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT status %d, service_handle %d",
                    param->start.status,
                    param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT conn_id = %d",
                    param->connect.conn_id);
            esp_log_buffer_hex(TAG, param->connect.remote_bda, 5);

            esp_ble_conn_update_params_t conn_params = {
                .latency = 0,
                .max_int = 0x20, // 0x20 * 1.25ms = 40ms
                .min_int = 0x10, // 0x10 * 1.25ms = 20ms
                .timeout = 1000, // 1000 * 10ms   = 10000ms
            };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

            esp_ble_gap_update_conn_params(&conn_params);
            oled_printf(4, " ES CONNECTED=%u", ++conn);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT reason = 0x%x",
                    param->disconnect.reason);
            // if the connection is in the notify list, remove it
            ble_conn_lst_remove(param->disconnect.conn_id);
            if (!(--conn))
                oled_printf(4, "                ");
            else
                oled_printf(4, " ES CONNECTED=%u", conn);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT code = 0x%x",
                        param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != IDX_SIZE){
                ESP_LOGE(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT (num_handle = %d) != (IDX_SIZE = %d)",
                        param->add_attr_tab.num_handle,
                        IDX_SIZE);
            }
            else {
                ESP_LOGI(TAG, "ESP_GATTS_CREAT_ATTR_TAB_EVT handle = %d",
                        param->add_attr_tab.num_handle);

                memcpy(nh3_handle_table, param->add_attr_tab.handles, sizeof(nh3_handle_table));
                esp_ble_gatts_start_service(nh3_handle_table[IDX_SVC]);
            }
            break;

        case ESP_GATTS_SET_ATTR_VAL_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_SET_ATTR_VAL_EVT");

            const uint8_t *value_ptr;
            uint16_t lentgh;

            ESP_ERROR_CHECK(esp_ble_gatts_get_attr_value(param->set_attr_val.attr_handle, &lentgh, &value_ptr));

            uint8_t value[4];
            memcpy(value, value_ptr, sizeof(value));

            ble_conn_lst_map(ble_notify_cb, value, sizeof(value));
            break;

        default:
            ESP_LOGI(TAG, "GATTS unhandled event %d", event);
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

    if((ble_adv_msg_queue = xQueueCreate(10, sizeof(char) * (18 + 1))) == NULL)
        ESP_LOGE(TAG, "Failed to crate ble_msg_queue");

    return ESP_OK;
}
