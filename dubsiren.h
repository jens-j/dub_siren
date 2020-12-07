#define PIN_POT         A1
#define PIN_I2S_BCLK    1
#define PIN_I2S_WCLK    2
#define PIN_I2S_DATA    3

#define CLOCK_RATE      48000000
#define SAMPLE_RATE     48000
#define OSC_FREQ_MIN    20
#define OSC_FREQ_MAX    20000
#define OSC_AMP_MAX     32767 // 16 bit DAC
#define OSC_BIAS        32768
#define LFO_FREQ_MIN    0.1
#define LFO_FREQ_MAX    10

#define QS15_ONE        0x7fff
#define QS15_MINUS_ONE  0x8000
#define QU16_ONE        0xffff


enum waveform_t {SQUARE, SAW, TRIANGLE, SINE};

typedef uint16_t qs15_t; // signed fixed point [-1 - 1)
typedef uint16_t qu16_t; // unsigned fixed point [0 - 1)

inline qs15_t qu16_to_qs15 (float x) {return (qs15_t) (x >> 1);}
inline qs15_t float_to_qs15 (float x) {return (qs15_t) (x * 0x8000);}
inline qu16_t float_to_qu16 (float x) {return (qu16_t) (x * 0x10000);}
inline qs15_t qs15_invert (qs15_t x) {return (x ^ 0xfff) + 1;}

// scale an integer by a qs15 fixed point value
inline uint16_t mul_qs15_uint16 (qs15_t x , uint16_t y) {

    // cast and sign extend x to 32 bits
    uint32_t temp_x = (uint32_t) x;
    temp_x |= 0xffff0000 * (temp_x >> 15);

    // multiply and normalize
    return (uint16_t) ((temp_x * y) >> 15);
}

// scale an integer by a qu16 fixed point value
inline uint16_t mul_qu16_uint16 (qu16_t x , uint16_t y) {

    // multiply and normalize
    return (uint16_t) (((uint32_t) x * y) >> 16);
}
