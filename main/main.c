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

#include "ble.h"
#include "dht.h"
#include "mics6814.h"
#include "ssd1306.h"
#include "utils.h"

#define PRG_GPIO 0
#define PRG_GPIO_PIN_SEL ((1ULL << PRG_GPIO))

#define DHT_GPIO 17

#define TASK_DELAY_MS(ms) (vTaskDelay(pdMS_TO_TICKS((ms))))

// log output to serial monitor
#define LOG_CSV

#define MICS_READ_TIME ((1000 * 1))
#define DHT_READ_TIME  ((1000 * 60))

SSD1306_t ssd1306_dev;

volatile bool mics6814_skip_warmup = false;

static TaskHandle_t dht_loop_handle;
static TaskHandle_t mics_loop_handle;

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

void dht_loop(void *args){
    int16_t t;
    int16_t h;

    while (1){
        if (dht_read_data((dht_sensor_type_t)DHT_TYPE_DHT11, (gpio_num_t)DHT_GPIO, &h, &t) == ESP_OK){

            oled_printf(2, " TEMP: %.1fC     ", t / 10.0f);
            oled_printf(4, " HUMD: %.1f%%    ", h / 10.0f);
        }
        else{
            oled_printf(2, "    DHT FAIL!   ");
            oled_printf(4, "    DHT FAIL!   ");
        }

        xTaskNotify(mics_loop_handle, (((uint32_t)t) << 16) | h, eSetValueWithOverwrite);

        TASK_DELAY_MS(DHT_READ_TIME);
    }
}

void mics_loop(void *args){
    uint32_t v;
    uint32_t n;
    uint16_t t = 0;
    uint16_t h = 0;

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
            oled_printf(0, " WARMING UP %lu ", MICS6814_WARMUP_TIME - time(NULL));
        }
        else{
            oled_printf(0, " SENSOR: %umV   ", v);
            ble_update_adv_msg("MICS %.1f %.1f %.1f", v / 1000.0f, t / 10.0f, h / 10.0f);
#ifdef LOG_CSV
            printf("%u,%.1f,%.1f\n", v, t / 10.0, h / 10.0); fflush(stdout);
#endif
        }

        TASK_DELAY_MS(MICS_READ_TIME);
    }
}

void app_main(void){
    // NOTE: i2c pins are defined using idf.py menucofig
    i2c_master_init(&ssd1306_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&ssd1306_dev, 128, 64);
    ssd1306_clear_screen(&ssd1306_dev, false);
    ssd1306_contrast(&ssd1306_dev, 0xFF);

    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    mics6814_init();

    prg_isr_init();

    oled_printf_init();

    ESP_ERROR_CHECK(ble_init());

    TASK_DELAY_MS(1000); // wait for the battery to stabilize

    xTaskCreatePinnedToCore(dht_loop, "dht_loop", 1792, NULL, 23, &dht_loop_handle, 1);
    xTaskCreatePinnedToCore(mics_loop, "mics_loop", 1792, NULL, 24, &mics_loop_handle, 1);
}
