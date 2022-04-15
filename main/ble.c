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

#define DEVICE_NAME "MICS-6814"

#define ESP_APP_ID 0x55
#define LOCAL_MTU  0x1F4

static QueueHandle_t ble_adv_msg_queue;

static uint8_t raw_adv_data[31] = {
    0x02, 0x0A, 0xEB,       // tx power
    0x03, 0x03, 0x2B, 0xCF, // 16-bit service uuid
    0x03, 0x19, 0x05, 0x42, // appearance values
    0x0A, 0x09, 'M', 'I', 'C', 'S', '-', '6', '8', '1', '4'
};

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_NONCONN_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

static
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    switch(event){
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
            ESP_LOGD(TAG, "ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT");
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            {
                char ble_adv_msg[18 + 1];
                size_t n;

                if (xQueueReceive(ble_adv_msg_queue, (void*)ble_adv_msg, (TickType_t)portMAX_DELAY) == pdPASS){

                    memcpy(&raw_adv_data[13], ble_adv_msg, (n = strlen(ble_adv_msg)));
                    raw_adv_data[11] = n + 1;

                    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));
                }
            }
            ESP_LOGD(TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
            break;
        default:
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

static
void gatts_event_handler(esp_gatts_cb_event_t event,
                         esp_gatt_if_t gatts_if,
                         esp_ble_gatts_cb_param_t *param){
    // for now only respond to register event
    switch (event){
        case ESP_GATTS_REG_EVT:
            ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));
            ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));
            ESP_LOGD(TAG, "ESP_GATTS_REG_EVT");
            break;
        default:
            break;
    }
}

inline __attribute__((always_inline))
esp_err_t ble_init(){
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
