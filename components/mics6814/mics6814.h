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
#define MICS6814_BETAS       3           // number of regression coefficients
#define MICS6814_LSTSQ       3

// NOTE: this is a big mess
#define MICS6814_ALLOC_BETAS(args...) do{ \
    mics6814_calibration_betas = malloc(sizeof(float) * MICS6814_BETAS); \
    if (mics6814_calibration_betas == NULL) ESP_LOGE(TAG, \
            "Fail to allocate memory for calibration coefficients"); \
    mics6814_initi_calibration(mics6814_calibration_betas, ##args); \
} while(0);

#define MICS6814_ALLOC_LSTSQ(args...) do{ \
    mics6814_calibration_lstsq = malloc(sizeof(float) * MICS6814_LSTSQ); \
    if (mics6814_calibration_lstsq == NULL) ESP_LOGE(TAG, \
            "fill to allocate memory for least squares coefficients"); \
    mics6814_initi_calibration(mics6814_calibration_lstsq, ##args); \
} while(0);

extern volatile bool mics6814_skip_warmup;
extern float *mics6814_calibration_betas;
extern float *mics6814_calibration_lstsq;

static __attribute__((unused)) // it is used, but for some reason gcc is bitching
void mics6814_initi_calibration(float *calibration_vector, ...){
    va_list args;
    va_start(args, calibration_vector);
    for (uint8_t i = 0; i < MICS6814_BETAS; i++)
        calibration_vector[i] = va_arg(args, double);
    va_end(args);
}

void mics6814_init();
uint32_t mics6814_read_voltage();
float mics6814_vraw_to_vreg(const uint32_t *, const int16_t *, const int16_t *);
float mics6814_to_ppm(const uint32_t*, const int16_t*, const int16_t*);

#endif
