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

static const char *TAG = "main";

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <esp_timer.h>
#include <esp_gatts_api.h>

#include "am2302.h"
#include "ble.h"
#include "mics6814.h"
#include "ssd1306.h"
#include "utils.h"

#define PRG_GPIO 0
#define PRG_GPIO_PIN_SEL ((1ULL << PRG_GPIO))

#define TASK_DELAY_MS(ms) (vTaskDelay(pdMS_TO_TICKS((ms))))

#define TASK_DELAY_UNTIL_INIT() TickType_t __prev = xTaskGetTickCount()
#define TASK_DELAY_UNTIL_MS(ms) do {               \
    vTaskDelayUntil(&__prev, pdMS_TO_TICKS((ms))); \
    __prev = xTaskGetTickCount();                  \
} while(0);

// log output to serial monitor
#undef LOG_CSV

#define MICS6814_READ_TIME ((1000 * 1))
#define AM2302_READ_TIME ((1000 * 60))

SSD1306_t ssd1306_dev;

volatile bool mics6814_skip_warmup = false;

static TaskHandle_t am2302_loop_handle;
static TaskHandle_t mics6814_loop_handle;

static
void IRAM_ATTR prg_isr(void *arg){
    mics6814_skip_warmup = !mics6814_skip_warmup;
}

static inline __attribute__((always_inline))
    void prg_isr_init(){
        gpio_config_t gpio = {
            .intr_type    = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = PRG_GPIO_PIN_SEL,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = true
        };

        gpio_config(&gpio);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(PRG_GPIO, prg_isr, (void*)PRG_GPIO);

        ESP_LOGI(TAG, "ISR installed on PRG button");
    }

void am2302_loop(void *args){
    int16_t t = 0;
    int16_t h = 0;

    uint8_t r = 0;

    // NOTE: this needs to happen here since this task is pinned to core 1,
    // if stating this on core 0 the first iteration fails, but it somehow
    // manages to fix itself, there is no documentation about this behaviour
    // NOTE: app_main should be pinned to core 1 as well...
    am2302_init();

    TASK_DELAY_MS(10); // NOTE: wait for the mics task to be created
    TASK_DELAY_UNTIL_INIT();
    while (1){
        if (am2302_read(&t, &h) == ESP_OK){
            oled_printf(2, " T=%.1fC H=%.1f%%   ", t / 10.0f, h / 10.0f);
            oled_printf(3, "                    ");
            r = 0;
        }
        else {
            r++;
            oled_printf(3, " T&H IS %ds OLD", r * AM2302_READ_TIME);
        }

        xTaskNotify(mics6814_loop_handle, (((uint32_t)t) << 16) | h, eSetValueWithOverwrite);
        TASK_DELAY_UNTIL_MS(AM2302_READ_TIME);
    }
}

void mics6814_loop(void *args){
    uint32_t v;
    uint32_t n;

    int16_t t = 0;
    int16_t h = 0;
    int16_t g = 0; // TODO: temperature and humidity gain

    mics6814_init();

    TASK_DELAY_UNTIL_INIT();

#ifdef LOG_CSV
    printf("SENSOR(mV),TEMPERATURE(C),HUMIDITY(%%)\n"); fflush(stdout);
#endif
    while (1){
        v = mics6814_read_voltage();

        if (xTaskNotifyWait(0, 0, &n, 0) == pdPASS){
            t = 0xFFFF & (n >> 16);
            h = 0xFFFF & n;
        }

        if (v >> 31){
            oled_printf(0, " WARMING UP %lu  ", MICS6814_WARMUP_TIME - time(NULL));
        }
        else{
            uint8_t value[4];

            uint16_t volt = v;

            for (uint16_t i = 0, k = 1000; i < 4; i++, k /= 10){
                value[i] = volt / k + 48;
                volt %= k;
            }

            oled_printf(0, " NH3=%uppm      ", v);
            oled_printf(1, " GAIN=%dmV      ", g);

            ble_update_adv_msg("MICS %.1f %.1f %.1f", v / 1000.0f, t / 10.0f, h / 10.0f);

            ble_set_nh3_attr(value, sizeof(value));

#ifdef LOG_CSV
            printf("%u,%.1f,%.1f\n", v, t / 10.0, h / 10.0); fflush(stdout);
#endif
        }

        TASK_DELAY_UNTIL_MS(MICS6814_READ_TIME);
    }
}

void app_main(void){
    // NOTE: ssd1306 i2c pins are defined using idf.py menucofig
    i2c_master_init(&ssd1306_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&ssd1306_dev, 128, 64);
    ssd1306_clear_screen(&ssd1306_dev, false);
    ssd1306_contrast(&ssd1306_dev, 0xFF);

    prg_isr_init();

    oled_printf_init();

    ESP_ERROR_CHECK(ble_init());

    // TASK_DELAY_MS(1000); // wait for the battery to stabilize

    xTaskCreatePinnedToCore(am2302_loop, "am2302_loop", 2048, NULL, 23, &am2302_loop_handle, 1);
    xTaskCreatePinnedToCore(mics6814_loop, "mics_loop", 2048, NULL, 24, &mics6814_loop_handle, 1);
}
