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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "dht.h"
#include "mics6814.h"
#include "ssd1306.h"

#define DHT_GPIO 17
#undef LOG_CSV

#define ADC  ADC1_CHANNEL_0 // pin 36
#define VREF 1100           // value calibrated on the chip, see debug log and menuconfig for the calibration method used

#define TASK_DELAY_MS(ms) (vTaskDelay(pdMS_TO_TICKS((ms))))

MICS6814_ADC_CHARACTERISTICS;
SSD1306_t dev;

// TODO: move this to another task, that updates the screen as needed
static inline
void oled_printf(SSD1306_t *dev, const bool invert, const uint8_t row, const char *restrict fmt, ...){
    char output[16] = {};
    va_list args;

    va_start(args, fmt);
    ssd1306_display_text(dev, row, output, vsnprintf(output, 16, fmt, args), invert);
    va_end(args);
}

void app_main(void){
    uint32_t volt; // ADC voltage
    int16_t tmp;
    int16_t hum;
    bool blink = 0;

    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xFF);

    mics6814_init();

    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    TASK_DELAY_MS(1000); // wait for the battery to stabilize

#ifdef LOG_CSV
    printf("SENSOR,TEMPERATURE,HUMIDITY\n"); fflush(stdout);
#endif

    while(1) {
        volt = mics6814_read_voltage();

        if (volt >> 31){
            oled_printf(&dev, blink, 0, " WARMING UP %lu ", MICS6814_WARMUP_TIME - time(NULL));
        }
        else{
            oled_printf(&dev, false, 0, " SENSOR: %umV   ", volt);
        }

        if (dht_read_data((dht_sensor_type_t)DHT_TYPE_AM2301, (gpio_num_t)DHT_GPIO, &hum, &tmp) == ESP_OK){
            oled_printf(&dev, false, 2, " TMP: %.1fC     ", tmp / 10.0f);
            oled_printf(&dev, false, 4, " HUM: %.1f%%    ", hum / 10.0f);
        }
        else{
            oled_printf(&dev, blink, 2, "    DHT FAIL!   ");
            oled_printf(&dev, blink, 4, "    DHT FAIL!   ");
        }
        blink = !blink;

#ifdef LOG_CSV
    printf("%u,%d,%d\n", volt, tmp, hum); fflush(stdout);
#endif

        TASK_DELAY_MS(1000);
    }
}
