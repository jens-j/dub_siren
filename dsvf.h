#include <math.h>
#include "dubsiren.h"
#include "fixedpoint.h"

#ifndef __DSVF_H
#define __DSVF_H

void get_dsvf_coeffs (float cutoff, float resonance, qs15_t *f, qs15_t *q) {

    *f = float_to_qs15(2 * sin(PI * cutoff / SAMPLE_RATE));
    *q = float_to_qs15(1 / resonance);
}

#endif
