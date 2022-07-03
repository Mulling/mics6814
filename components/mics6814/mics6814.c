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

static const char *TAG = "mics6814";

#include <esp_adc_cal.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include <time.h>

#include "mics6814.h"

static esp_adc_cal_characteristics_t mics6814_adc_characteristics;

static inline __attribute__((always_inline))
void mics6814_init_adc(){
    // adc2_vref_to_gpio((gpio_num_t)25); // set to read VREF

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(MICS6814_ADC_CHANNEL, ADC_ATTEN_11db);

    esp_adc_cal_value_t mode =
        esp_adc_cal_characterize(MICS6814_ADC, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                                 MICS6814_ADC_VREF, &mics6814_adc_characteristics);

    switch (mode){
        case ESP_ADC_CAL_VAL_EFUSE_VREF:
            ESP_LOGI(TAG, "ADC eFuse Vref used for characterization");
            break;
        case ESP_ADC_CAL_VAL_EFUSE_TP:
            ESP_LOGI(TAG, "ADC Two Point value used for characterization");
            break;
        case ESP_ADC_CAL_VAL_DEFAULT_VREF:
            ESP_LOGI(TAG, "ADC Default Vref used for characterization");
            break;
        default:
            break;
    }
}

inline __attribute__((always_inline))
void mics6814_init(){
    mics6814_init_adc();

    if (mics6814_calibration_betas == NULL) ESP_LOGE(TAG, "Fail to set calibration betas");

    ESP_LOGI(TAG, "Sensor started");
}

uint32_t mics6814_read_voltage(){
    uint32_t ret = 0;
    // NOTE: the voltage should never get this high, so doing this *should* be fine
    if (!mics6814_skip_warmup && (time(NULL) <= (time_t)MICS6814_WARMUP_TIME)) return 0x80000000;

    for (uint8_t i = 0; i < MICS6814_SAMPLE_SIZE; i++)
        ret += adc1_get_raw((adc_channel_t)MICS6814_ADC_CHANNEL);

    return esp_adc_cal_raw_to_voltage((ret >> MICS6814_SAMPLE) & 0xFFF, &mics6814_adc_characteristics);
}

inline __attribute__((always_inline))
float mics6814_vraw_to_vreg(const uint32_t *vraw, const int16_t *temp, const int16_t *humd){
    return (float)*vraw - (mics6814_calibration_betas[0]
            + (mics6814_calibration_betas[1] * (*temp) / 10.0)
            + (mics6814_calibration_betas[2] * (*humd) / 10.0));
}

inline __attribute__((always_inline))
float mics6814_to_ppm(const uint32_t *vraw, const int16_t *temp, const int16_t *humd){
    float vreg = mics6814_vraw_to_vreg(vraw, temp, humd);
    return (mics6814_calibration_lstsq[0] * mics6814_calibration_lstsq[0] * vreg)
        + (mics6814_calibration_lstsq[1] * vreg)
        + mics6814_calibration_lstsq[2];
}
