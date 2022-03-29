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

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "dht.h"
#include "mics6814.h"
#include "ssd1306.h"

#define PPG_GPIO 0
#define PPG_GPIO_PIN_SEL ((1ULL << PPG_GPIO))
#define DHT_GPIO 17
#define ADC  ADC1_CHANNEL_0 // pin 36
#define VREF 1106           // read from adc2_vref_to_gpio, see debug log and menuconfig for the calibration method used

#define TASK_DELAY_MS(ms) (vTaskDelay(pdMS_TO_TICKS((ms))))

// log output to serial monitor
#undef LOG_CSV

MICS6814_ADC_CHARACTERISTICS;
SSD1306_t ssd1306_dev;

QueueHandle_t oled_text_row_queue;
TaskHandle_t ssd1306_drawline_loop_handle;

volatile bool mics6814_skip_warmup = false;

struct oled_text_row {
    int row;
    char msg[16];
};

static
void IRAM_ATTR ppg_isr(void *arg){
    (void)arg;
    mics6814_skip_warmup = !mics6814_skip_warmup;
}

static
void oled_printf(const uint8_t row, const char *restrict fmt, ...){
    struct oled_text_row oled_row = {.row = row};

    va_list args;

    va_start(args, fmt);
    vsnprintf(oled_row.msg, 16, fmt, args);
    va_end(args);

    xQueueSend(oled_text_row_queue, (void*)&oled_row, (TickType_t)1);

    xTaskNotify(ssd1306_drawline_loop_handle, 0, eNoAction);
}

void ssd1306_drawrow_loop(void *args){
    struct oled_text_row oled_row;

    while(true){
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        // try to consume all from the queue
        while (xQueueReceive(oled_text_row_queue, (void*)&oled_row, (TickType_t)1) == pdPASS){
            ssd1306_display_text(&ssd1306_dev, oled_row.row, oled_row.msg, 16, false);
        }
    }
}

void app_main(void){
    uint32_t volt; // ADC voltage
    int16_t tmp;
    int16_t hum;

    gpio_config_t gpio;
    gpio.intr_type = GPIO_INTR_NEGEDGE;
    gpio.pin_bit_mask = PPG_GPIO_PIN_SEL;
    gpio.mode = GPIO_MODE_INPUT;
    gpio.pull_up_en = true;
    gpio_config(&gpio);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PPG_GPIO, ppg_isr, (void*)PPG_GPIO);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    i2c_master_init(&ssd1306_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&ssd1306_dev, 128, 64);
    ssd1306_clear_screen(&ssd1306_dev, false);
    ssd1306_contrast(&ssd1306_dev, 0xFF);

    mics6814_init();

    oled_text_row_queue = xQueueCreate(10, sizeof(struct oled_text_row));

    xTaskCreatePinnedToCore(ssd1306_drawrow_loop, "ssd1306_drawrow_loop", 1024, NULL, 1, &ssd1306_drawline_loop_handle, 1);

    TASK_DELAY_MS(1000); // wait for the battery to stabilize

#ifdef LOG_CSV
    printf("SENSOR,TEMPERATURE,HUMIDITY\n"); fflush(stdout);
#endif

    while(true) {
        volt = mics6814_read_voltage();

        if (volt >> 31){
            oled_printf(0, " WARMING UP %lu ", MICS6814_WARMUP_TIME - time(NULL));
        }
        else{
            oled_printf(0, " SENSOR: %umV   ", volt);
        }

        if (dht_read_data((dht_sensor_type_t)DHT_TYPE_AM2301, (gpio_num_t)DHT_GPIO, &hum, &tmp) == ESP_OK){
            oled_printf(2, " TMP: %.1fC     ", tmp / 10.0f);
            oled_printf(4, " HUM: %.1f%%    ", hum / 10.0f);
        }
        else{
            oled_printf(2, "    DHT FAIL!   ");
            oled_printf(4, "    DHT FAIL!   ");
        }

#ifdef LOG_CSV
        if (!(volt >> 31))
            printf("%u,%d,%d\n", volt, tmp, hum); fflush(stdout);
#endif

        TASK_DELAY_MS(1000);
    }
}
