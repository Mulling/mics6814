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

static const char *TAG = "oprintf";

#include "utils.h"

#include <esp_log.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdint.h>

#include "ssd1306.h"

#define SSD1306_CHAR_LINE 16

static QueueHandle_t oprintf_loop_queue;
static TaskHandle_t opritnf_loop_handle;

struct oled_text_row {
    uint8_t row;
    char msg[SSD1306_CHAR_LINE + 1];
};

static
void oprintf_loop(void *args){
    struct oled_text_row oled_row;

    while (1){
        // try to consume all from the queue
        while (xQueueReceive(oprintf_loop_queue, (void *)&oled_row, (TickType_t)portMAX_DELAY) == pdPASS){
            ssd1306_display_text(&ssd1306_dev, oled_row.row, oled_row.msg, SSD1306_CHAR_LINE , false);
        }
    }
}

inline __attribute__((always_inline))
void oprintf_init(){
    if ((oprintf_loop_queue = xQueueCreate(10, sizeof(struct oled_text_row))) == NULL)
        ESP_LOGE(TAG, "fail to create oprintf_loop_queue");

    xTaskCreatePinnedToCore(oprintf_loop, "oprintf_loop", 1024, NULL, 1, &opritnf_loop_handle, 1);
}

void oprintf(const uint8_t row, const char *restrict fmt, ...){
    struct oled_text_row oled_row = {.row = row};

    va_list args;

    va_start(args, fmt);
    vsnprintf(oled_row.msg, SSD1306_CHAR_LINE + 1, fmt, args);
    va_end(args);

    xQueueSend(oprintf_loop_queue, (void *)&oled_row, (TickType_t)0);
}
