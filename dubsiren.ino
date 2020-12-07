#include <I2S.h>
#include <sam.h>
#include "dubsiren.h"
#include "sine.h"


volatile waveform_t osc_waveform = SINE;
volatile int osc_period          = SAMPLE_RATE / 440; // in samples per period
volatile qu16_t osc_resolution   = float_to_qu16_t(1 / osc_period); // sample resolution normalized to number of periods
volatile qu16_t osc_phase        = 0; // always positive [0 - 1]
volatile int osc_amplitude       = OSC_AMP_MAX / 10;

volatile waveform_t lfo_waveform = SINE;
volatile int lfo_period          = SAMPLE_RATE;
volatile qu16_t lfo_resolution   = float_to_qu16_t(1 / lfo_period);
volatile qu16_t lfo_phase        = 0;
volatile int lfo_amplitude       = 1;


void setup () {
    pinMode(PIN_I2S_BCLK, OUTPUT);
    pinMode(PIN_I2S_WCLK, OUTPUT);
    pinMode(PIN_I2S_DATA, OUTPUT);

    Serial.begin(115200);

    if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16)) {
        Serial.println("Failed to initialize I2S");
        while (1); // do nothing
    }

    // enable APB bus clock for I2C and TC3 modules
    PM_APBCMASK      = PM_APBCMASK_TC3 | PM_APBCMASK_I2S;

    // set TC3 to generate 48kHz interrupts
    REG_TC3_CTRLC    = TC_CTRLC_CPTEN0; // enable capture on channel 0 (necessary in MFRQ mode?)
    REG_TC3_INTENSET = TC_CTRLC_MC0;    // enable capture channel 0 interrupt
    REG_TC3_CC0      = 1000;            // set period to 1000 cycles for a 48kHz frequency
    REG_TC3_CTRLA    = TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_ENABLE;  // 16-bit mode, no prescaler, MFRQ mode, enable timer

    // Enable TC3 interrupt in the NVIC
    NVIC_ISER        = 1 << IRQn[TC3_IRQn];
}


void loop () {
    // read frequency from pot
    delay(1)
}


// return the amplitude in [-1 - 1]
qs15_t inline getAmplitude(waveform_t waveform, qu16_t phase) {

    switch (waveform) {
        case SQUARE:
            return (phase < QU16_ONE / 2) ? QS15_ONE : QS15_MINUS_ONE;

        // the phase to amplitude ratio has a gain of two. Because of the fixed point limits of
        // [-1, 1) a multiplication could overflow. Therefore the phase is just subtracted twice.
        case SAW:
           return QS15_ONE - qu16_to_qs15(phase) - qu16_to_qs15(phase);

        case TRIANGLE:
            qs15_t local_phase;
            if (phase < QU16_ONE / 2) {
                local_phase = qu16_to_qs15(phase) << 2; // [0 - 0.5] -> [0 - 1]
                return QS15_MINUS_ONE + local_phase + local_phase;
            } else {
                local_phase = (qu16_to_qs15(phase) - (QS15_ONE >> 2)) << 2; // [0.5 - 1] -> [0 - 1]
                return QS15_ONE - local_phase - local_phase;
            }

        case SINE:
            // TODO: interpolation
            uint16_t index = mul_qu16_uint16(phase, SINE_SAMPLES * 4) & (SINE_SAMPLES - 1);
            if (phase < QU16_ONE / 4) {
                return SINE_TABLE[index];
            } else if (phase < QU16_ONE / 2) {
                return SINE_TABLE[SINE_SAMPLES - index - 1];
            } else if (phase < QU16_ONE * 3 / 4) {
                return qs15_invert(SINE_TABLE[index]);
            } else {
                return qs15_invert(SINE_TABLE[SINE_SAMPLES - index - 1]);
            }
    }
}




void TC3_Handler() {

    uint16_t sample;

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_resolution;
    lfo_phase += lfo_resolution;

    // get oscillator value
    sample = OSC_BIAS + osc_amplitude * getAmplitude(osc_waveform, osc_phase);

    // write sample to DAC for both channels
    I2S.write(sample);
    I2S.write(sample);
}
