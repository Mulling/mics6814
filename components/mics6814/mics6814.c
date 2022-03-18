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

#include <esp_adc_cal.h>
#include <esp_rom_sys.h>
#include <stdint.h>

#include "mics6814.h"

void mics6814_init_adc(){
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(MICS6814_ADC_CHANNEL, ADC_ATTEN_11db);

    esp_adc_cal_characterize(MICS6814_ADC,
                             ADC_ATTEN_DB_11,
                             ADC_WIDTH_BIT_12,
                             MICS6814_ADC_VREF,
                             &mics6814_adc_characteristics);
}

uint32_t mics6814_read_voltage(){
    uint32_t ret;
    uint32_t aux;

    esp_adc_cal_get_voltage((adc_channel_t)MICS6814_ADC_CHANNEL, &mics6814_adc_characteristics, &ret);

    for (uint8_t i = 0; i < MICS6814_SAMPLE_SIZE - 1; i++){
        esp_adc_cal_get_voltage((adc_channel_t)MICS6814_ADC_CHANNEL, &mics6814_adc_characteristics, &aux);
        ret += aux;
    }

    return ret >> 3;
}
