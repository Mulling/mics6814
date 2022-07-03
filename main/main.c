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
#define TASK_DELAY_UNTIL_MS(ms) do { \
    vTaskDelayUntil(&__prev, pdMS_TO_TICKS((ms))); \
    __prev = xTaskGetTickCount(); \
} while(0);

// log output to serial monitor
#define LOG_CSV

#define MICS6814_READ_TIME ((1000 * 1)) // 1 second
#define AM2302_READ_TIME   ((1000 * 30)) // 30 seconds

#define SENSOR_BLE_MSG_FMT       "MICS-NH3@%uppm"
#define SENSOR_RAW_FMT           "RAW=%umV"
#define SENSOR_REG_FMT           "NH3=%umV"
#define SENSOR_PPM_FMT           "NH3=%uppm"
#define SENSOR_WARMUP_FMT        "WARMING UP %lu"
#define TEMPERATURE_HUMIDITY_FMT "T=%.1fC H=%.1f%%"

SSD1306_t ssd1306_dev;

volatile bool mics6814_skip_warmup = false;

float *mics6814_calibration_betas = NULL;
float *mics6814_calibration_lstsq = NULL;

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
    gpio_isr_handler_add(PRG_GPIO, prg_isr, (void *)PRG_GPIO);

    ESP_LOGI(TAG, "ISR installed on PRG button");
}

void am2302_loop(void *args){
    int16_t t = 0;
    int16_t h = 0;

    // NOTE: this needs to happen here since this task is pinned to core 1,
    // if stating this on core 0 the first iteration fails, but it somehow
    // manages to fix itself, there is no documentation about this behaviour
    am2302_init();

    TASK_DELAY_MS(10); // NOTE: wait for the mics task to be created

    TASK_DELAY_UNTIL_INIT();
    while (1){
        if (am2302_read(&t, &h) == ESP_OK){
            oprintf(2, TEMPERATURE_HUMIDITY_FMT, t / 10.0f, h / 10.0f);

            xTaskNotify(mics6814_loop_handle, (((uint32_t)t) << 16) | h, eSetValueWithOverwrite);
        }

        TASK_DELAY_UNTIL_MS(AM2302_READ_TIME);
    }
}

static inline __attribute__((always_inline))
uint16_t to_sfloat16(const uint32_t bits){
    // NOTE: IEEE-11073 SFLOAT, source: Personal Health Devices Transcoding WP
    // +-----------------+-----------------+
    // | 4 bits exponent | 12 bit mantissa |
    // +-----------------+-----------------+

    // TODO: fix this mess
    uint8_t  e = 0x0;
    // NOTE: it's fine doing this since the concentration
    // *should* never be higher than 500ppm (< 2^12)
    return ((e << 12) & 0xF000) | (bits & 0x0FFF);
}

static inline __attribute__((always_inline))
void ppm_to_string(char *str, uint32_t cppm){
    for (uint8_t i = 0, k = 100; i < 3; i++, k /= 10){
        str[i] = cppm / k + 48;
        cppm %= k;
    }
}

void mics6814_loop(void *args){
    uint32_t vraw; // raw voltage reading
    uint32_t vreg; // corrected voltage for temperature and humidity
    uint32_t cppm;

    float fppm;

    uint32_t n;

    int16_t t = 0;
    int16_t h = 0;

    MICS6814_ALLOC_BETAS(1323.56724301, -26.46680528, -9.315224);
    MICS6814_ALLOC_LSTSQ(0.00005043, -0.33039421, 501.52061563);

    mics6814_init();

    TASK_DELAY_UNTIL_INIT();
    while (1){
        if (xTaskNotifyWait(0, 0, &n, 0) == pdPASS){
            t = 0xFFFF & (n >> 16);
            h = 0xFFFF & n;
        }

        if ((vraw = mics6814_read_voltage()) >> 31){
            oprintf(0, SENSOR_WARMUP_FMT, MICS6814_WARMUP_TIME - time(NULL));
            oprintf(1, "");
        } else {
            vreg = (uint32_t)mics6814_vraw_to_vreg(&vraw, &t, &h);
            fppm = mics6814_to_ppm(&vraw, &t, &h);
            cppm = fppm <= 0 ? 0 : (uint32_t)fppm;
            cppm %= 500;

            char sppm[3];
            ppm_to_string(sppm, cppm);
            ble_set_nh3_attr((uint8_t *)&cppm, sizeof(uint8_t) * 2);

            uint16_t vreg_sfloat16 = to_sfloat16(cppm);

            ble_update_adv_msg(vreg_sfloat16, SENSOR_BLE_MSG_FMT, cppm);

            oprintf(0, SENSOR_RAW_FMT, vraw);
            oprintf(1, SENSOR_REG_FMT, vreg);
            oprintf(6, SENSOR_PPM_FMT, cppm);

#ifdef LOG_CSV
            printf("%u,%.1f,%.1f,%u,%u\n", vraw, t / 10.0, h / 10.0, vreg, cppm);
#endif
        }
        TASK_DELAY_UNTIL_MS(MICS6814_READ_TIME);
    }
}

void app_main(void){
    // NOTE: ssd1306 i2c pins are defined using idf.py menuconfig
    i2c_master_init(&ssd1306_dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);

    ssd1306_init(&ssd1306_dev, 128, 64);
    ssd1306_clear_screen(&ssd1306_dev, false);
    ssd1306_contrast(&ssd1306_dev, 0xFF);

    prg_isr_init();

    oprintf_init();

    ESP_ERROR_CHECK(ble_init());

    // TASK_DELAY_MS(1000); // wait for the battery to stabilize

    xTaskCreatePinnedToCore(am2302_loop, "am2302_loop", 2048, NULL, 23, &am2302_loop_handle, 1);
    xTaskCreatePinnedToCore(mics6814_loop, "mics_loop", 2048, NULL, 24, &mics6814_loop_handle, 1);
}
