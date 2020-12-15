//#include <I2S.h>
#include <sam.h>
#include <wiring_private.h>
#include "dubsiren.h"
#include "sine.h"


volatile waveform_t osc_waveform = SINE;
volatile qu32_t osc_phase        = 0;                // always positive [0 - 1]
volatile uint16_t osc_amplitude  = 5000;
volatile qu16_t osc_setpoint     = uint16_to_qu16(1000);             // base frequency setpoint, set by PIN_OSC_POT
volatile qu16_t osc_frequency    = osc_setpoint;     // base frequency current value
volatile uint16_t mod_frequency  = 0;                // additive frequency shift from lfo
volatile qu32_t osc_step         = osc_setpoint * (QU32_ONE / SAMPLE_RATE); // 1000 Hz phase change per sample

volatile waveform_t lfo_waveform = SQUARE;
volatile qu8_t lfo_frequency     = float_to_qu8(2.0);
volatile qu32_t lfo_step         = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE); // phase change per sample

volatile qu32_t lfo_phase        = 0;
volatile qs15_t lfo_mod_depth    = QS15_ONE / 10;    // mod osc frequency by 10%
volatile uint16_t sample         = 0;                // buffer one sample to handle interrupts quickly
volatile bool btn_state          = false;

uint32_t led_t0                  = 0;
bool led_state                   = true;
bool pwm_state                   = true;
bool last_btn_state              = false;
uint16_t osc_reading             = 0;
uint16_t lfo_reading             = 0;


void setup () {
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_I2S_BCLK, OUTPUT);
    pinMode(PIN_I2S_WCLK, OUTPUT);
    pinMode(PIN_I2S_DATA, OUTPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for a serial connection (terminal)

    setupAdc();
    setupI2S();

    Serial.println(GCLK->CLKCTRL.reg);
    Serial.println("Initialization completed");
}


inline uint16_t readAdc (int channel) {

    ADC->INPUTCTRL.bit.MUXPOS = channel;         // ADC+ to channel ANx
    ADC->SWTRIG.bit.START = 1;
    while (!ADC->INTFLAG.bit.RESRDY);
    return ADC->RESULT.reg;
}

void setupAdc () {

    GCLK->CLKCTRL.bit.ID = 0x1e;                // select clock GCLK_ADC
    GCLK->CLKCTRL.bit.GEN = 3;                  // clock generator 4
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    PORT->Group[1].PINCFG[2].bit.PMUXEN = 1;    // mux ADC on PB02 / pin A1
    PORT->Group[0].PINCFG[4].bit.PMUXEN = 1;    // mux ADC on PA04 / pin A3
    PORT->Group[1].PMUX[1].bit.PMUXE = 1;       // select AN10 (group B) for PB02
    PORT->Group[0].PMUX[2].bit.PMUXE = 1;       // select AN04 (group B) for PA04

    ADC->CTRLA.bit.ENABLE = 0;                  // disable peripheral before starting clock
    PM->APBCMASK.bit.ADC_ = 1;                  // start APBC clock

    ADC->CTRLB.bit.PRESCALER = 0;               // 4
    ADC->CTRLB.bit.RESSEL = 1;                  // 16 bit
    ADC->INPUTCTRL.bit.MUXNEG = 0x18;           // ADC- to internal ground
    ADC->AVGCTRL.bit.SAMPLENUM = 7;             // 128 samples
    ADC->AVGCTRL.bit.ADJRES = 6;
    ADC->SAMPCTRL.bit.SAMPLEN = 0x3f;

    ADC->CTRLA.bit.ENABLE = 1;
}

void setupI2S() {
    // set the I2S module to 48 kHz tx 16-bit mono mode

    // setup the clock for the I2S peripheral
    GCLK->GENDIV.bit.ID = 3;                    // select generator 3
    GCLK->GENDIV.bit.DIV = 31;                  // clock divider 48 MHz / 31 / 16 / 2 = 48.387 kHz
    GCLK->GENCTRL.bit.ID = 3;                   // select generator 3
    GCLK->GENCTRL.bit.SRC = 7;                  // FDPLL48M
    GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    GCLK->CLKCTRL.bit.ID = 0x23;                // select clock GCLK_I2S_0
    GCLK->CLKCTRL.bit.GEN = 3;                  // clock generator 3
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    // enable peripheral mux
    PORT->Group[0].PINCFG[7].bit.PMUXEN = 1;
    PORT->Group[0].PINCFG[10].bit.PMUXEN = 1;
    PORT->Group[0].PINCFG[11].bit.PMUXEN = 1;

    // set port mux values
    PORT->Group[0].PMUX[3].bit.PMUXO = 6;       // data -> PA7
    PORT->Group[0].PMUX[5].bit.PMUXE = 6;       // sclk -> PA10
    PORT->Group[0].PMUX[5].bit.PMUXO = 6;       // wclk -> PA11

    I2S->CTRLA.bit.ENABLE = 0;                  // disable peripheral before starting clock
    PM->APBCMASK.bit.I2S_ = 1;                  // start APBC clock

    // setup clock unit 0
    I2S->CLKCTRL[0].bit.BITDELAY = 1;           // use I2S stadard
    I2S->CLKCTRL[0].bit.NBSLOTS = 1;            // 2 slots per frame
    I2S->CLKCTRL[0].bit.SLOTSIZE = 1;           // 16 bit

    // setup serial unit 0
    // I2S->SERCTRL[0].bit.MONO = 1; // this still generates a TXRDY0 for both samples
    I2S->SERCTRL[0].bit.DATASIZE = 5;           // 16 bit compact stereo
    I2S->SERCTRL[0].bit.SLOTADJ = 1;            // left justified
    I2S->SERCTRL[0].bit.SERMODE = 1;            // tx mode
    I2S->SERCTRL[0].bit.TXSAME = 1;             // repeat last word on underflow

    I2S->INTENSET.bit.TXRDY0 = 1;               // enable tx ready interrupt

    NVIC_ClearPendingIRQ(I2S_IRQn);
    NVIC_SetPriority(I2S_IRQn, 1);
    NVIC_EnableIRQ(I2S_IRQn);

    // enable clock unit, serial unit and peripheral
    I2S->CTRLA.bit.CKEN0 = 1;
    I2S->CTRLA.bit.SEREN0 = 1;
    I2S->CTRLA.bit.ENABLE = 1;
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
                local_phase = (qu32_to_qs15(phase) - (QS15_ONE >> 2)); // [0.5 - 1] -> [0 - 1]
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
                return qs15_invert(SINE_TABLE[index]);
            } else {
                return qs15_invert(SINE_TABLE[SINE_SAMPLES - index - 1]);
            }
    }
}


void I2S_Handler() {

    // write sample in compact stereo mode, interrupt flag is cleared automatically
    I2S->DATA[0].reg = btn_state ? (uint32_t) sample << 16 | sample : 0UL;

    if (I2S->INTFLAG.bit.TXUR0 == 1) {
        I2S->INTFLAG.bit.TXUR0 = 1;
        digitalWrite(PIN_PWM, pwm_state);
        pwm_state = !pwm_state;
    }

    // update oscillator frequency (glide)
    qs15_t osc_frequency_diff = qu16_to_qs15(osc_setpoint) - qu16_to_qs15(osc_frequency);
    if (osc_frequency_diff & 0x80000000) {
        osc_frequency -= min(qs15_invert(osc_frequency_diff), GLIDE_RATE);
    } else {
        osc_frequency += min(osc_frequency_diff, GLIDE_RATE);
    }

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_step;
    lfo_phase += lfo_step;

    // get lfo value
    qs15_t lfo_value = getAmplitude(lfo_waveform, lfo_phase);

    mod_frequency = mul_qs15_uint16(
        mul_qs15(lfo_value, lfo_mod_depth), qu16_to_uint16(osc_frequency));
    osc_step = (uint16_t) (qu16_to_uint16(osc_frequency) + mod_frequency)
        * (QU32_ONE / SAMPLE_RATE);

    // get oscillator value
    sample = mul_qs15_uint16(getAmplitude(osc_waveform, osc_phase), osc_amplitude);
}

void loop () {

    char stringBuffer[100];

    // read button
    if (digitalRead(PIN_BTN)) {
        btn_state = false;
    } else {
        if (!last_btn_state) {lfo_phase = 0;} // reset phase at start of envelope
        btn_state = true;
    }
    last_btn_state = btn_state;

    // read frequency pot
    int new_osc_reading = readAdc(ADC_CH_OSC);
    if (abs(osc_reading - new_osc_reading) > 1) {
        osc_reading = new_osc_reading;
        qu16_t norm_osc_reading = uint16_to_qu16(osc_reading) >> ADC_RES_LOG2; // normalize reading to [0 - 1)
        norm_osc_reading = mul_qu16(norm_osc_reading, norm_osc_reading); // create quadratic curve
        osc_setpoint = norm_osc_reading * OSC_FREQ_RANGE + uint16_to_qu16(OSC_FREQ_MIN); // calculate frequency setpoint
    }

    // read lfo pot
    int new_lfo_reading = readAdc(ADC_CH_LFO);
    if (abs(lfo_reading - new_lfo_reading) > 2) {
        lfo_reading = new_lfo_reading;
        lfo_frequency = float_to_qu8((float) lfo_reading / ADC_RES * LFO_FREQ_RANGE + LFO_FREQ_MIN);
        lfo_step = mul_qu8_uint32(lfo_frequency, QU32_ONE / SAMPLE_RATE);
    }

    // blink led at 2 Hz
    int t = millis();
    if (t - led_t0 > MAIN_LOOP_MS) {
        digitalWrite(PIN_LED, led_state);
        led_state = !led_state;
        led_t0 += MAIN_LOOP_MS;

        // qs15_t osc_frequency_diff = qu16_to_qs15(osc_setpoint) - qu16_to_qs15(osc_frequency);
        // sprintf(stringBuffer, "%x, %x %x %x",
        //     qu16_to_qs15(osc_setpoint),
        //     qu16_to_qs15(osc_frequency),
        //     osc_frequency_diff,
        //     min(osc_frequency_diff, GLIDE_RATE));
        //
        // Serial.println(stringBuffer);


        Serial.println((float) lfo_phase / 4294967296);
    }
}
