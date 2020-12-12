//#include <I2S.h>
#include <sam.h>
#include <wiring_private.h>
#include "dubsiren.h"
#include "sine.h"


volatile waveform_t osc_waveform = SINE;
volatile uint16_t osc_frequency  = 1000;                    // output frequency
volatile qu32_t osc_step         =                          // phase change per sample
    osc_frequency * (QU32_ONE / SAMPLE_RATE);
volatile qu32_t osc_phase        = 0;                       // always positive [0 - 1]
volatile uint16_t osc_amplitude  = 5000;
volatile uint16_t osc_setting    = 0;                       // base frequency set by PIN_OSC_POT
volatile uint16_t mod_frequency  = 0;                       // additive frequency shift from lfo


volatile waveform_t lfo_waveform = SINE;
volatile uint16_t lfo_frequency  = 1;                    // output frequency
volatile uint32_t lfo_period     = SAMPLE_RATE * 2;
volatile qu32_t lfo_step         = QU16_ONE / lfo_period;
volatile qu32_t lfo_phase        = 0;
volatile qs15_t lfo_mod_depth    = QS15_ONE / 10;
volatile uint16_t sample         = 0;                       // buffer one sample to handle interrupts quickly

uint32_t led_t0                  = 0;
bool led_state                   = true;
bool pwm_state                   = true;
uint16_t osc_reading             = 0;


void setup () {
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_I2S_BCLK, OUTPUT);
    pinMode(PIN_I2S_WCLK, OUTPUT);
    pinMode(PIN_I2S_DATA, OUTPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for a serial connection (terminal)
    //I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16);e

    setupAdc();
    setupI2S();

    Serial.println(GCLK->CLKCTRL.reg);
    Serial.println("Initialization completed");
}

void loop () {

    // read frequency pot
    int new_osc_reading = readAdc();
    if (abs(osc_reading - new_osc_reading) > 2) {
        osc_frequency = int(map(new_osc_reading, 0, 1024, OSC_FREQ_MIN, OSC_FREQ_MAX));
        osc_step = osc_frequency * (QU32_ONE / SAMPLE_RATE);
        osc_reading = new_osc_reading;
    }

    // blink led at 2 Hz
    int t = millis();
    if (t - led_t0 > 1000) {
        digitalWrite(PIN_LED, led_state);
        led_state = !led_state;
        led_t0 = t;

        Serial.println(I2S->SERCTRL[0].reg, HEX);
    }
}

inline uint16_t readAdc () {
    ADC->SWTRIG.bit.START = 1;
    while (!ADC->INTFLAG.bit.RESRDY);
    return ADC->RESULT.reg;
}

void setupAdc () {
    //ADC->CTRLA.bit.ENABLE = 0;

    // setup the clock for the ADC peripheral
    // GCLK->GENDIV.bit.ID = 4;                    // select generator 4
    // GCLK->GENDIV.bit.DIV = 64;                  // c
    // GCLK->GENCTRL.bit.ID = 4;                   // select generator 4
    // GCLK->GENCTRL.bit.SRC = 7;                  // FDPLL48M
    // GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    // GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    GCLK->CLKCTRL.bit.ID = 0x1e;                // select clock GCLK_ADC
    GCLK->CLKCTRL.bit.GEN = 3;                  // clock generator 4
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    PORT->Group[1].PINCFG[2].bit.PMUXEN = 1;    // mux ADC on PB02 / pin A1
    PORT->Group[1].PMUX[1].bit.PMUXE = 1;       // select AN10 (group B) for in PB02

    ADC->CTRLA.bit.ENABLE = 0;                  // disable peripheral before starting clock
    PM->APBCMASK.bit.ADC_ = 1;                  // start APBC clock

    ADC->CTRLB.bit.PRESCALER = 0;               // 4
    ADC->CTRLB.bit.RESSEL = 1;                  // 16 bit
    ADC->INPUTCTRL.bit.MUXNEG = 0x18;           // ADC- to internal ground
    ADC->INPUTCTRL.bit.MUXPOS = 0x0A;           // ADC+ to AN10
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
                digitalWrite(PIN_PWM, HIGH);
                local_phase = qu32_to_qs15(phase) << 2; // [0 - 0.5] -> [0 - 1]
                return QS15_MINUS_ONE + local_phase;
            } else {
                digitalWrite(PIN_PWM, LOW);
                local_phase = (qu32_to_qs15(phase) - (QS15_ONE >> 2)) << 2; // [0.5 - 1] -> [0 - 1]
                return -local_phase;
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
    // this routine is called twice for every sample because mono mode does not

    // write sample, interrupt flag is cleared automatically
    I2S->DATA[0].reg = sample << 16 | sample;

    // if (osc_phase + osc_step < osc_phase) {
    // digitalWrite(PIN_PWM, pwm_state);
    // pwm_state = !pwm_state;
    // }

    // update oscillator and lfo phase. these overflow naturally
    osc_phase += osc_step;
    lfo_phase += lfo_step;

    // get lfo value
    // qs15_t lfo_value = getAmplitude(lfo_waveform, lfo_phase);

    // // modulate osc frequency with scaled lfo value
    // mod_frequency = mul_qs15_uint16(mul_qs15(lfo_value, lfo_mod_depth), osc_setting);
    // osc_frequency = osc_setting + mod_frequency;
    // osc_period = SAMPLE_RATE / osc_frequency;
    // osc_step = QU16_ONE / osc_period;

    // get oscillator value
    sample = mul_qs15_uint16(getAmplitude(osc_waveform, osc_phase), osc_amplitude);
}
