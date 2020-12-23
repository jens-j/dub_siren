#include <stdint.h>

#ifndef __DUBSIREN_H
#define __DUBSIREN_H

#define PIN_LED         32
#define PIN_PWM         5
#define PIN_BTN         7
#define PIN_OSC_POT     A1
#define PIN_LFO_DEPTH   A2
#define PIN_LFO_POT     A3
#define PIN_LFO_SHAPE   A4
#define PIN_DECAY       A5
#define PIN_I2S_BCLK    2
#define PIN_I2S_WCLK    3
#define PIN_I2S_DATA    A6

#define ADC_CH_LFO      4
#define ADC_CH_SHAPE    5
#define ADC_CH_DECAY    6
#define ADC_CH_OSC      10
#define ADC_CH_DEPTH    11

#define CLOCK_RATE      48000000UL
#define MAIN_LOOP_MS    200
#define ADC_RES         1024
#define ADC_RES_LOG2    10
#define SAMPLE_RATE     48000UL
#define POT_DEAD_ZONE   2

#define OSC_FREQ_MIN    100
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

#define DECAY_MAX       5.0 // seconds

#define QU32_ONE        0xffffffffUL
#define QU16_ONE        0x0000ffffUL
#define QS15_ONE        0x00007fffUL
#define QS15_MINUS_ONE  0xffff8000UL


enum waveform_t {SQUARE, SAW, TRIANGLE, SINE};

typedef uint32_t qu8_t;  // unsigned 24.8 fixed point
typedef int32_t qs15_t;  // signed fixed point [-1 - 1)
typedef uint32_t qu16_t; // unsigned fixed point [0 - 1)
typedef uint32_t qu32_t; // unsigned fixed point [0 - 1)

inline qu16_t qu32_to_qu16 (qu32_t x) {return (qu16_t) (x >> 16);}
inline qs15_t qu32_to_qs15 (qu32_t x) {return (qs15_t) (x >> 17);}
inline qs15_t qu16_to_qs15 (qu16_t x) {return (qs15_t) (x >> 1);}
inline qu8_t qu16_to_qu8 (qu16_t x) {return (qu8_t) (x >> 8);}
inline qu16_t uint16_to_qu16 (uint16_t x) {return (qu16_t) (x << 16);}
inline qs15_t uint16_to_qs15 (uint16_t x) {return (qs15_t) (x << 15);}
inline qu8_t uint16_to_qu8 (uint16_t x) {return (qu8_t) (x << 8);}
inline uint16_t qu16_to_uint16 (qu16_t x) {return (uint16_t) (x >> 16);}
inline qu8_t float_to_qu8 (float x) {return (qu8_t) (x * 0x100);}
inline qs15_t float_to_qs15 (float x) {return (qs15_t) (x * 0x8000);}
inline qu16_t float_to_qu16 (float x) {return (qu16_t) (x * 0x10000);}
inline qu32_t float_to_qu32 (float x) {return (qu32_t) (x * 0x100000000);}
inline qs15_t qs15_invert (qs15_t x) {return (~x) + 1;}


// multiply two qu16_t values
inline qu16_t mul_qu16 (qu16_t x, qu16_t y) {

    return x * y >> 16;
}

// multiply two qu8_t values
inline qu8_t mul_qu8 (qu8_t x, qu8_t y) {

    return x * y >> 8;
}

// multiply two qs15_t values
inline qs15_t mul_qs15 (qs15_t x, qs15_t y) {

    return x * y >> 15;
}

// scale an int by a signed coefficient
inline uint16_t mul_qs15_uint16 (qs15_t x, uint16_t y) {

    return (uint16_t) (x * (uint32_t) y >> 15);
}

// scale an integer by a unsigned coefficient
inline uint16_t mul_qu16_uint16 (qu16_t x, uint16_t y) {

    return (uint16_t) (x * (uint32_t) y >> 16);
}

// scale an integer by a unsigned coefficient
inline uint32_t mul_qu8_uint32 (qu8_t x, uint32_t y) {

    return (x * y) >> 8;
}

// scale an integer by a qu32 fixed point value
inline uint16_t mul_qu32_uint16 (qu32_t x, uint16_t y) {

    return mul_qu16_uint16(qu32_to_qu16(x), y);
}


#endif
