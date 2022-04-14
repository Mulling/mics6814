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

#include <stdint.h>

#define MICS6814_ADC         ADC_UNIT_1
#define MICS6814_ADC_CHANNEL ADC1_CHANNEL_0
#define MICS6814_ADC_VREF    1106        // read using adc2_vref_to_gpio
#define MICS6814_SAMPLE_SIZE 16          // 2^n
#define MICS6814_SAMPLE      4           // log2(MICS6814_SAMPLE_SIZE)
#define MICS6814_WARMUP_TIME ((60 * 60)) // 1 hour
#define MICS6814_WARMUP      0x80000000  // sensor warm-up

extern volatile bool mics6814_skip_warmup;

void mics6814_init();
uint32_t mics6814_read_voltage();

#endif
