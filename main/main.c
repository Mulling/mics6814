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

#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "dht.h"
#include "mics6814.h"

#define DHT_GPIO 17

#define ADC  ADC1_CHANNEL_0 // pin 36
#define VREF 1100           // value calibrated on the chip

#define TASK_DELAY_MS(ms) (vTaskDelay(pdMS_TO_TICKS((ms))))

esp_adc_cal_characteristics_t mics6814_adc_characteristics;

void app_main(void) {
    uint32_t volt; // ADC voltage
    int16_t tmp;
    int16_t hum;

    mics6814_init_adc();

    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);

    while(1) {
        volt = mics6814_read_voltage();

        printf("Sensor voltage: %d\n", volt);

        if (dht_read_data((dht_sensor_type_t)DHT_TYPE_AM2301, (gpio_num_t)DHT_GPIO, &hum, &tmp) == ESP_OK)
            printf("Humidity: %d%%\nTemp: %dC\n", hum, tmp);
        else
            printf("Could not read data from sensor\n");

        TASK_DELAY_MS(5000);
    }
}
