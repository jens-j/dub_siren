//#include <I2S.h>
#include <sam.h>
#include <wiring_private.h>
#include "dubsiren.h"
#include "sine.h"


volatile waveform_t osc_waveform = SINE;
volatile int osc_period          = SAMPLE_RATE / 1000; // in samples per period
volatile qu16_t osc_resolution   = float_to_qu16(1.0 / osc_period); // sample resolution normalized to number of periods
volatile qu16_t osc_phase        = 0; // always positive [0 - 1]
volatile int osc_amplitude       = 5000;

volatile waveform_t lfo_waveform = SINE;
volatile int lfo_period          = SAMPLE_RATE;
volatile qu16_t lfo_resolution   = float_to_qu16(1.0 / lfo_period);
volatile qu16_t lfo_phase        = 0;
volatile int lfo_amplitude       = 1;

volatile bool led_state          = true;
volatile bool pwm_state          = true;

volatile uint16_t sample         = 0;


void setup () {
    // pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_I2S_BCLK, OUTPUT);
    pinMode(PIN_I2S_WCLK, OUTPUT);
    pinMode(PIN_I2S_DATA, OUTPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for a serial connection (terminal)
    //I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16);e

    setupI2S();
    //setupTC3();

    Serial.println("Initialization completed");
}

void loop () {
    // Serial.println("tik");
    Serial.println(I2S->SERCTRL[0].reg, HEX);
    digitalWrite(PIN_LED, led_state);
    led_state = !led_state;
    delay(1000);
}

void setupI2S() {
    // set the I2S module to 48 kHz tx 16-bit mono mode

    // setup the clock for the I2S peripheral
    GCLK->GENDIV.bit.ID = 3;                    // select generator 0
    GCLK->GENDIV.bit.DIV = 62;                  // clock divider 96 MHz / 125 / 16 = 48 kHz
    GCLK->GENCTRL.bit.ID = 3;                   // select generator 0
    GCLK->GENCTRL.bit.SRC = 7;                  // FDPLL48M
    GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    GCLK->CLKCTRL.bit.ID = 0x23;                // select clock GCLK_I2S_0
    GCLK->CLKCTRL.bit.GEN = 3;                  // clock generator 0
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    PM->APBCMASK.bit.I2S_ = 1;

    // enable peripheral mux
    PORT->Group[0].PINCFG[7].bit.PMUXEN = 1;
    PORT->Group[0].PINCFG[10].bit.PMUXEN = 1;
    PORT->Group[0].PINCFG[11].bit.PMUXEN = 1;

    // set port mux values
    PORT->Group[0].PMUX[3].bit.PMUXO = 6;       // data -> PA7
    PORT->Group[0].PMUX[5].bit.PMUXE = 6;       // sclk -> PA10
    PORT->Group[0].PMUX[5].bit.PMUXO = 6;       // wclk -> PA11

    pinPeripheral(PIN_I2S_BCLK, PIO_COM);
    pinPeripheral(PIN_I2S_WCLK, PIO_COM);
    pinPeripheral(PIN_I2S_DATA, PIO_COM);

    I2S->CTRLA.bit.ENABLE = 0;                  // disable peripheral before starting clock
    PM->APBCMASK.bit.I2S_ = 1;                  // start APBC clock

    // setup clock unit 0
    I2S->CLKCTRL[0].bit.BITDELAY = 1;           // use I2S stadard
    I2S->CLKCTRL[0].bit.NBSLOTS = 1;            // 2 slots per frame
    I2S->CLKCTRL[0].bit.SLOTSIZE = 1;           // 16 bit

    // setup serial unit 0
    I2S->SERCTRL[0].bit.MONO = 1;
    I2S->SERCTRL[0].bit.DATASIZE = 4;           // 16 bit
    I2S->SERCTRL[0].bit.SLOTADJ = 1;            // left justified
    I2S->SERCTRL[0].bit.SERMODE = 1;            // tx mode

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
qs15_t inline getAmplitude(waveform_t waveform, qu16_t phase) {

    switch (waveform) {
        case SQUARE:
            //return (phase < QU16_ONE / 4) ? QS15_ONE : QS15_MINUS_ONE;

            if (phase < QU16_ONE / 4) {
                return qu16_to_qs15(QU16_ONE);
            } else {
                return QS15_MINUS_ONE;
            }

        // the phase to amplitude ratio has a gain of two. Because of the fixed point limits of
        // [-1, 1) a multiplication could overflow. Therefore the phase is just subtracted twice.
        case SAW:
           return QS15_ONE - qu16_to_qs15(phase) - qu16_to_qs15(phase);
           // return qu16_to_qs15(phase);

        case TRIANGLE:
            qs15_t local_phase;
            if (phase < QU16_ONE / 2) {
                local_phase = qu16_to_qs15(phase) << 2; // [0 - 0.5] -> [0 - 1]
                return QS15_MINUS_ONE + local_phase;
            } else {
                local_phase = (qu16_to_qs15(phase) - (QS15_ONE >> 2)) << 2; // [0.5 - 1] -> [0 - 1]
                return QS15_ONE - local_phase;
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


void I2S_Handler() {

    // write sample, interrupt flag is cleared automatically
    I2S->DATA[0].reg = sample;

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_resolution;
    lfo_phase += lfo_resolution;

    // get oscillator value
    sample = mul_qs15_uint16(getAmplitude(osc_waveform, osc_phase), osc_amplitude);
}


void TC3_Handler() {

    // clear interrupt flag
    TC3->COUNT16.INTFLAG.bit.MC0 = 1;

    uint16_t sample;

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_resolution;
    lfo_phase += lfo_resolution;

    /// get oscillator value
    sample = mul_qs15_uint16(getAmplitude(osc_waveform, osc_phase), osc_amplitude);

    // I2S->DATA[0].reg = sample;
}
