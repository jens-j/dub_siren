#ifndef __DUBSIREN_H
#define __DUBSIREN_H

#define PIN_BOARD_LED       32
#define PIN_LFO_LED         6
#define PIN_SW_SUB          14
#define PIN_SW_RANGE        A3
#define PIN_SW_SWEEP        0
#define PIN_TRIGGER         12
#define PIN_ADC_CH0         A5
#define PIN_ADC_CH1         A4
#define PIN_I2S_BCLK        2
#define PIN_I2S_WCLK        3
#define PIN_I2S_DATA        A6
#define PIN_SPI_CLK         9
#define PIN_SPI_MISO        10
#define PIN_SPI_MOSI        8
#define PIN_SPI_SS          7
#define PIN_SREG_CLK        15
#define PIN_SREG_DATA       17
#define PIN_SREG_LATCH      16
#define PIN_MUX_S0          5
#define PIN_MUX_S1          4
#define PIN_MUX_S2          1

#define GLCK_I2S            3
#define GCLK_ADC            4
#define GLCK_SPI            5
#define GLCK_TCC            6

#define CLOCK_RATE          48000000UL
#define PRINT_MS            500
#define ADC_RES             1024
#define ADC_RES_LOG2        10
#define SAMPLE_RATE         44117UL
#define POT_DEAD_ZONE       3
#define BTN_TIME            20 // ms

#define OSC_FREQ_MIN        20.0
#define OSC_FREQ_MAX        1000.0
#define OSC_FREQ_RANGE      (OSC_FREQ_MAX - OSC_FREQ_MIN)
#define OSC_AMP_MAX         32767 // 16 bit signed
#define OSC_AMP             2500
#define OSC_SWEEP_MAX       (OSC_FREQ_RANGE / 2.0) // change / s
#define GLIDE_RATE          (QS15_ONE >> 2) // frequency change rate in Hz/sample

#define LFO_FREQ_MIN        0.2
#define LFO_FREQ_MAX_LOW    16.0
#define LFO_FREQ_MAX_HIGH   187.0 // higher than this overflows somehow
#define LFO_FREQ_RANGE_LOW  (LFO_FREQ_MAX_LOW - LFO_FREQ_MIN)
#define LFO_FREQ_RANGE_HIGH (LFO_FREQ_MAX_HIGH - LFO_FREQ_MIN)
#define LFO_DEPTH_MIN       0.0
#define LFO_DEPTH_MAX       4.0
#define LFO_DEPTH_RANGE     (LFO_DEPTH_MAX - LFO_DEPTH_MIN)

#define FILTER_MIN          100.0 // lower leads to unstable filter due to coefficient rounding
#define FILTER_MAX          8000.0
#define FILTER_RANGE        (FILTER_MAX - FILTER_MIN)
#define RESONANCE_MIN       1.0 // lower leads to unstable filter due to coefficient rounding
#define RESONANCE_MAX       8.0
#define RESONANCE_RANGE     (RESONANCE_MAX - RESONANCE_MIN)
#define FILTER_SWEEP_MAX    OSC_FREQ_RANGE // change / s

#define DECAY_MAX           5.0 // seconds

#define DELAY_WET_MAX       0.5
#define DELAY_TIME_MIN      1 // in SPI block index
#define DELAY_TIME_MAX      (RAM_BYTES / SPI_BLOCK_BYTES) // ~1.365 s
#define DELAY_TIME_RANGE    (DELAY_TIME_MAX - DELAY_TIME_MIN)
#define DELAY_FEEDBACK_MAX  0.5

#define RAM_BYTES           (1 << 17)
#define RAM_SAMPLES         (RAM_BYTES >> 1)
#define RAM_BLOCKS          (RAM_BYTES / SPI_BLOCK_SIZE)

#define SPI_BLOCK_SIZE      1024
#define SPI_BLOCK_BYTES     (SPI_BLOCK_SIZE << 1)
#define SPI_BUFFER_BYTES    (SPI_BLOCK_BYTES + 5)

#define PI                  3.141593

enum waveform_t {SINE, SINE_H3, CHORD, SAW_UP, SAW_DOWN, SAW_WHOOP, CAPACITOR,
                 SQUARE, PULSE, LASER_SAW, LASER_SQUARE, RANDOM};

#endif
