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

#ifndef MICS6814_H
#define MICS6814_H

#include <esp_adc_cal.h>
#include <esp_rom_sys.h>
#include <stdint.h>

#define MICS6814_ADC         ADC_UNIT_1
#define MICS6814_ADC_CHANNEL ADC1_CHANNEL_0
#define MICS6814_ADC_VREF    1100
#define MICS6814_SAMPLE_SIZE 8

extern esp_adc_cal_characteristics_t mics6814_adc_characteristics;

void mics6814_init_adc();
uint32_t mics6814_read_voltage();

#endif
