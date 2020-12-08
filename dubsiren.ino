#include <I2S.h>
#include <sam.h>
#include "dubsiren.h"
#include "sine.h"


volatile waveform_t osc_waveform = SINE;
volatile int osc_period          = SAMPLE_RATE / 440; // in samples per period
volatile qu16_t osc_resolution   = float_to_qu16(1 / osc_period); // sample resolution normalized to number of periods
volatile qu16_t osc_phase        = 0; // always positive [0 - 1]
volatile int osc_amplitude       = OSC_AMP_MAX / 10;

volatile waveform_t lfo_waveform = SINE;
volatile int lfo_period          = SAMPLE_RATE;
volatile qu16_t lfo_resolution   = float_to_qu16(1 / lfo_period);
volatile qu16_t lfo_phase        = 0;
volatile int lfo_amplitude       = 1;

bool led_state                   = true;
volatile bool pwm_state          = true;


void setup () {
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_PWM, OUTPUT);
    // pinMode(PIN_I2S_BCLK, OUTPUT);
    // pinMode(PIN_I2S_WCLK, OUTPUT);
    // pinMode(PIN_I2S_DATA, OUTPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for a serial connection (terminal)

    // if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16)) {
    //     Serial.println("Failed to initialize I2S");
    //     //while (1); // do nothing
    // }

    // PORT->Group[0].PMUX[9].bit.PMUXO     = 0x4;  // mux function E (TC/TCC) to PA19 (WO1)
    // PORT->Group[0].PINCFG[19].bit.PMUXEN = 0x1; // PA19 Mux ON

    GCLK->CLKCTRL.bit.ID = 0x1B;             // Clock selection (TC3)
    GCLK->CLKCTRL.bit.GEN = 0x0;             // Clock generator 0
    GCLK->CLKCTRL.bit.CLKEN = 0x1;           // Enable

    PM->APBCMASK.bit.TC3_ = 0x1;             // enable APB bus clock for TC3 module

    TC3->COUNT16.INTENSET.bit.MC0 = 1;       // enable capture channel 0 interrupt

    TC3->COUNT16.CTRLA.bit.ENABLE = 0;       // disable TC3
    TC3->COUNT16.CTRLA.bit.PRESCSYNC = 0x1;  // 48MHz
    TC3->COUNT16.CTRLA.bit.MODE = 0x00;      // 16 Bit Timer
    TC3->COUNT16.CTRLA.bit.WAVEGEN = 0x3;    // MPWM
    TC3->COUNT16.CTRLA.bit.PRESCALER = 0x00; // 1
    TC3->COUNT16.CC[0].bit.CC = 1000;        // capture every 1000 cycle -> 48 kHz

    NVIC_ClearPendingIRQ(TC3_IRQn);
    NVIC_SetPriority(TC3_IRQn, 1);
    NVIC_EnableIRQ(TC3_IRQn);

    TC3->COUNT16.CTRLA.bit.ENABLE = 1;       // enable TC3

    Serial.println("Initialization completed");
}


void loop () {
    Serial.println("tik");
    digitalWrite(PIN_LED, led_state);
    led_state = !led_state;
    delay(1000);
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

    // clear interrupt flag
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;

    digitalWrite(PIN_PWM, pwm_state);
    pwm_state = !pwm_state;



    // uint16_t sample;
    //
    // // update oscillator and lfo phase. these overflow naturally
    // osc_phase += osc_resolution;
    // lfo_phase += lfo_resolution;
    //
    // // get oscillator value
    // sample = OSC_BIAS + osc_amplitude * getAmplitude(osc_waveform, osc_phase);
    //
    // // write sample to DAC for both channels
    // I2S.write(sample);
    // I2S.write(sample);
}
