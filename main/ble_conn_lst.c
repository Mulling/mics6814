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

static const char __attribute__((unused)) *TAG = "ble_conn";

#include "ble_conn_lst.h"

#include <esp_gatts_api.h>
#include <esp_log.h>
#include <stdint.h>
#include <stdio.h>

#include "utils.h"

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

    oled_printf(5, " ES NOTIFY=%u", size + 1);
}

inline __attribute__((always_inline))
void ble_conn_lst_remove(const uint16_t conn_id){
    ble_conn **head = &ble_conn_lst;

    while ((*head)){
        if ((*head)->conn_id == conn_id){
            ble_conn *old = *head;
            *head = (*head)->next;
            free(old);
            if ((*head) == NULL) oled_printf(5, "                ");
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
