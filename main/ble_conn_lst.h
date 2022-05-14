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

#ifndef BLE_CONN_LST_H
#define BLE_CONN_LST_H

#include <esp_gatts_api.h>
#include <stdint.h>

typedef struct ble_conn {
    uint16_t conn_id;
    esp_gatt_if_t gatts_if;
    struct ble_conn *next;
} ble_conn;

// NOTE: list of connections with notify enabled
extern ble_conn *ble_conn_lst;

void ble_conn_lst_insert(const esp_gatt_if_t, const uint16_t);
void ble_conn_lst_remove(const uint16_t);
void ble_conn_lst_map(void(*fn)(const esp_gatt_if_t, const uint16_t, uint8_t*, const uint16_t), uint8_t*, const uint16_t);

#endif
