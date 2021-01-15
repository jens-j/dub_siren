//#include <I2S.h>
#include <sam.h>
#include <wiring_private.h>
#include "dubsiren.h"
#include "input.h"
#include "fixedpoint.h"
#include "spidma.h"
#include "i2s.h"
#include "sine.h"
#include "biquad.h"

Input input                      = Input();
uint32_t led_t0                  = 0;
bool led_state                   = true;
bool pwm_state                   = true;
bool last_btn_state              = false;

volatile waveform_t osc_waveform = SQUARE;
volatile qu32_t osc_phase        = 0;                // always positive [0 - 1]
volatile uint16_t osc_amplitude  = 5000;
volatile qu16_t osc_setpoint     = uint16_to_qu16(1000); // base frequency setpoint, set by PIN_OSC_POT
volatile qu16_t osc_frequency    = osc_setpoint;     // base frequency current value
volatile uint16_t mod_frequency  = 0;                // additive frequency shift from lfo
volatile qu32_t osc_step         = osc_setpoint * (QU32_ONE / SAMPLE_RATE); // 1000 Hz phase change per sample

volatile waveform_t lfo_shape    = SQUARE;
volatile qu8_t lfo_frequency     = float_to_qu8(2.0);
volatile qu32_t lfo_step         = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE); // phase change per sample
volatile qu32_t lfo_phase        = 0;
volatile qs15_t lfo_depth        = QS15_ONE / 10;    // mod osc frequency by 10%

volatile qu8_t decay_time        = 0;
volatile qu32_t decay_value      = QU32_ONE;
volatile qu32_t decay_step       = 0;

volatile uint16_t cutoff         = FILTER_MAX;
volatile qs12_t filter_a[2]      = {0, 0};
volatile qs12_t filter_b[3]      = {0, 0, 0};
volatile uint16_t filter_in[3]   = {0, 0, 0};        // unfiltered
volatile uint16_t filter_out[3]  = {0, 0, 0};        // filtered

volatile bool btn_state          = false;
volatile uint16_t sample         = 0;                // buffer one sample to handle interrupts quickly
volatile uint32_t loop_t0        = 0;
volatile uint32_t loop_t1        = 0;


void setup () {
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_I2S_BCLK, OUTPUT);
    pinMode(PIN_I2S_WCLK, OUTPUT);
    pinMode(PIN_I2S_DATA, OUTPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for a serial connection (terminal)

    setupSpi();
    setupI2S();

    Serial.println("Initialization completed");
}


// return the amplitude in [-1 - 1]
qs15_t inline getAmplitude(waveform_t waveform, qu32_t phase) {

    switch (waveform) {
        case SQUARE:
            //return (phase < QU16_ONE / 4) ? QS15_ONE : QS15_MINUS_ONE;
            if (phase < QU32_ONE / 2) {
                return QS15_ONE;
            } else {
                return QS15_MINUS_ONE;
            }

        // the phase to amplitude ratio has a gain of two. Because of the fixed point limits of
        // [-1, 1) a multiplication could overflow. Therefore the phase is just subtracted twice.
        case SAW:
           return QS15_ONE - qu32_to_qs15(phase) - qu32_to_qs15(phase);
           // return qu16_to_qs15(phase);

        case TRIANGLE:
            qs15_t local_phase;
            if (phase < QU32_ONE / 2) {
                local_phase = qu32_to_qs15(phase);
                return QS15_MINUS_ONE + (local_phase << 2);
            } else {
                local_phase = (qu32_to_qs15(phase) - (QS15_ONE >> 1)); // [0.5 - 1] -> [0 - 1]
                return QS15_ONE - (local_phase << 2);
            }

        case SINE:
            // TODO: interpolation
            uint16_t index = mul_qu32_uint16(phase, SINE_SAMPLES * 4) & (SINE_SAMPLES - 1);
            if (phase < QU32_ONE / 4) {
                return SINE_TABLE[index];
            } else if (phase < QU32_ONE / 2) {
                return SINE_TABLE[SINE_SAMPLES - index - 1];
            } else if (phase < QU32_ONE / 4 * 3) {
                return qs_invert(SINE_TABLE[index]);
            } else {
                return qs_invert(SINE_TABLE[SINE_SAMPLES - index - 1]);
            }
    }
}


void I2S_Handler() {
    // write sample in compact stereo mode, interrupt flag is cleared automatically
    //I2S->DATA[0].reg = btn_state ? (uint32_t) sample << 16 | sample : 0UL;
    I2S->DATA[0].reg = (uint32_t) filter_out[0] << 16 | filter_out[0];

    // shift sample buffers
    filter_in[2] = filter_in[1];
    filter_in[1] = filter_in[0];
    filter_out[2] = filter_out[1];
    filter_out[1] = filter_out[0];

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_step;
    lfo_phase += lfo_step;

    // decrease decay coefficient
    if (!btn_state) {
        if (decay_step < decay_value) {
            decay_value -= decay_step;
        } else {
            decay_value = 0;
        }
    }

    // update oscillator frequency (glide)
    qs15_t osc_frequency_diff = qu16_to_qs15(osc_setpoint) - qu16_to_qs15(osc_frequency);
    if (osc_frequency_diff & 0x80000000) {
        osc_frequency -= min(qs_invert(osc_frequency_diff), GLIDE_RATE);
    } else {
        osc_frequency += min(osc_frequency_diff, GLIDE_RATE);
    }

    // get lfo value
    qs15_t lfo_value = getAmplitude(lfo_shape, lfo_phase);

    mod_frequency = mul_qs15_int16(mul_qs15(lfo_value, lfo_depth), qu16_to_uint16(osc_frequency));
    osc_step = (uint16_t) (qu16_to_uint16(osc_frequency) + mod_frequency) * (QU32_ONE / SAMPLE_RATE);

    qs15_t amplitude = getAmplitude(osc_waveform, osc_phase);
    qs15_t decay_coeff = qu32_to_qs15(decay_value);
    decay_coeff = mul_qs15(decay_coeff, decay_coeff); // make quadratic

    // multiply the wavoform with the decay coefficient and use that to scale the amplitude
    filter_in[0] = mul_qs15_int16(amplitude, osc_amplitude);
    filter_in[0] = mul_qs15_int16(decay_coeff, filter_in[0]);

    // apply biquad filter
    filter_out[0] = mul_qs12_int16(filter_b[0], filter_in[0])
                    + mul_qs12_int16(filter_b[1], filter_in[1])
                    + mul_qs12_int16(filter_b[2], filter_in[2])
                    + mul_qs12_int16(filter_a[0], filter_out[1])
                    + mul_qs12_int16(filter_a[1], filter_out[2]);
}

void loop () {

    char stringBuffer[100];

    loop_t0 = millis();

    // read button
    if (digitalRead(PIN_BTN)) {
        btn_state = false;
    } else {
        if (!last_btn_state) {lfo_phase = 0;} // reset phase at start of envelope
        btn_state = true;
        decay_value = QU32_ONE;
    }
    last_btn_state = btn_state;

    input.update();

    // read frequency pot
    qu16_t norm_osc_reading = uint16_to_qu16(input.osc_frequency) >> ADC_RES_LOG2; // normalize reading to [0 - 1)
    norm_osc_reading = mul_qu16(norm_osc_reading, norm_osc_reading); // create quadratic curve
    osc_setpoint = norm_osc_reading * OSC_FREQ_RANGE + uint16_to_qu16(OSC_FREQ_MIN); // calculate frequency setpoint

    // read lfo pot
    qu16_t norm_lfo_reading = uint16_to_qu16(input.lfo_frequency) >> ADC_RES_LOG2;
    norm_lfo_reading = mul_qu16(norm_lfo_reading, norm_lfo_reading);
    lfo_frequency = mul_qu8(qu16_to_qu8(norm_lfo_reading), float_to_qu8(LFO_FREQ_RANGE))
        + float_to_qu8(LFO_FREQ_MIN);
    lfo_step = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE);

    // read lfo mod depth pot
    qs15_t norm_depth_reading = uint16_to_qs15(input.lfo_depth) >> ADC_RES_LOG2;
    lfo_depth = mul_qs15_int16(norm_depth_reading, float_to_qs15(LFO_DEPTH_RANGE))
        + float_to_qs15(LFO_DEPTH_MIN);

    // read decay pot
    qu16_t norm_decay_reading = uint16_to_qu16(input.decay_time) >> ADC_RES_LOG2;
    decay_time = mul_qu8(qu16_to_qu8(norm_decay_reading), float_to_qu8(DECAY_MAX));
    decay_step = QU32_ONE / mul_qu8_uint32(decay_time, SAMPLE_RATE);

    // // read lfo shape pot
    // int new_shape_reading = readAdc(ADC_CH_SHAPE);
    // if (abs(shape_reading - new_shape_reading) > POT_DEAD_ZONE) {
    //     shape_reading = new_shape_reading;
    //     if (shape_reading >= ADC_RES * 3 / 4) {
    //         lfo_shape = SQUARE;
    //     } else if (shape_reading >= ADC_RES / 2) {
    //         lfo_shape = SAW;
    //     } else if (shape_reading >= ADC_RES / 4) {
    //         lfo_shape = TRIANGLE;
    //     } else {
    //         lfo_shape = SINE;
    //     }
    // }

    cutoff = (uint16_t)
        ((((uint32_t) input.filter_cutoff * FILTER_RANGE) >> ADC_RES_LOG2) + FILTER_MIN);
    get_lpf_coeffs(cutoff, 4, filter_a, filter_b);

    loop_t1 = millis() - loop_t0;

    // blink led at 2 Hz
    int t = millis();
    if (t - led_t0 > MAIN_LOOP_MS) {

        digitalWrite(PIN_LED, led_state);
        led_state = !led_state;
        led_t0 += MAIN_LOOP_MS;

        Serial.println(loop_t1);
        Serial.println("");
    }
}
