#ifndef __DUBSIREN_H
#define __DUBSIREN_H

#define PIN_LED         32
#define PIN_PWM         5
#define PIN_BTN         6
#define PIN_OSC_POT     A1
#define PIN_LFO_DEPTH   A2
#define PIN_LFO_POT     A3
#define PIN_LFO_SHAPE   A4
#define PIN_DECAY       A5
#define PIN_I2S_BCLK    2
#define PIN_I2S_WCLK    3
#define PIN_I2S_DATA    A6
#define PIN_SPI_CLK     9
#define PIN_SPI_MISO    10
#define PIN_SPI_MOSI    8
#define PIN_SPI_SS      7

#define ADC_CH_LFO      4
#define ADC_CH_SHAPE    5
#define ADC_CH_FILTER   5
#define ADC_CH_DECAY    6
#define ADC_CH_OSC      10
#define ADC_CH_DEPTH    11

#define GLCK_I2S        3
#define GCLK_ADC        4
#define GLCK_SPI        5

#define CLOCK_RATE      48000000UL
#define MAIN_LOOP_MS    500
#define ADC_RES         1024
#define ADC_RES_LOG2    10
#define SAMPLE_RATE     48000UL
#define POT_DEAD_ZONE   2
#define BTN_TIME        20 // ms

#define OSC_FREQ_MIN    20
#define OSC_FREQ_MAX    10000
#define OSC_FREQ_RANGE  (OSC_FREQ_MAX - OSC_FREQ_MIN)
#define OSC_AMP_MAX     32767 // 16 bit signed
#define GLIDE_RATE      (QS15_ONE >> 2) // frequency change rate in Hz/sample

#define LFO_FREQ_MIN    0.2
#define LFO_FREQ_MAX    64.0
#define LFO_FREQ_RANGE  (LFO_FREQ_MAX - LFO_FREQ_MIN)
#define LFO_DEPTH_MIN   0.0
#define LFO_DEPTH_MAX   0.9
#define LFO_DEPTH_RANGE (LFO_DEPTH_MAX - LFO_DEPTH_MIN)

#define FILTER_MIN      500 // lower leads to unstable filter due to coefficient rounding
#define FILTER_MAX      OSC_FREQ_MAX
#define FILTER_RANGE    (FILTER_MAX - FILTER_MIN)

#define DECAY_MAX       5.0 // seconds

#define PI              3.141593

enum waveform_t {SQUARE, SAW, TRIANGLE, SINE};

inline uint16_t add_uint16_clip(uint16_t x, uint16_t y) {
    uint32_t z = (uint32_t) x * y;
    return z & 0x10000 ? 0xFFFF : (uint16_t) z;
}

#endif
