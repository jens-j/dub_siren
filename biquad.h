#include "dubsiren.h"
#include "fixedpoint.h"
#include "sine.h"

#ifndef __BIQUAD_H
#define __BIQUAD_H

// should not overflow to about 10430 Hz cutoff
void get_lpf_coeffs (uint16_t f, uint16_t q, volatile qs12_t *a, volatile qs12_t *b) {

    qs12_t sin_w, cos_w;
    uint16_t index = mul_qu16_uint16( // one lut equals 1/2 pi w
        uint16_to_qu16(f) / SAMPLE_RATE, SINE_SAMPLES * 4) & (SINE_SAMPLES - 1);

    // only the first quarter of the sine and cosine will be used due to the 10430 limit
    sin_w = qs15_to_qs12(SINE_TABLE[index]);
    cos_w = qs15_to_qs12(SINE_TABLE[SINE_SAMPLES - index - 1]);
    qs12_t alpha = sin_w / (q << 1);
    qs12_t a0 = QS12_ONE + alpha;

    b[0] = (QS12_ONE - cos_w) >> 1;
    b[1] = QS12_ONE - cos_w;
    a[0] = cos_w << 1; // invert a coefficients
    a[1] = alpha - QS12_ONE;

    // normalize coefficients
    b[0] = div_qs12(b[0], a0);
    b[1] = div_qs12(b[1], a0);
    b[2] = b[0];
    a[0] = div_qs12(a[0], a0);
    a[1] = div_qs12(a[1], a0);
}

#endif
