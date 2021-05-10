#include "Arduino.h"
#include "dubsiren.h"
#include "fixedpoint.h"
#include "input.h"


pot_route_t POT_ROUTING[10] = {
    {5, 1}, // POT_OSC_FREQ
    {5, 2}, // POT_LFO_DEPTH
    {5, 3}, // POT_LFO_FREQ
    {6, 7}, // POT_RELEASE
    {6, 6}, // POT_FILTER_FREQ
    {6, 5}, // POT_FILTER_RES
    {6, 0}, // POT_FILTER_SWEEP
    {6, 3}, // POT_DELAY_WET
    {6, 4}, // POT_DELAY_TIME
    {6, 2} // POT_DELAY_FB
};


Input::Input () {
    osc_waveform = SINE;
    _setupAdc();
    update();
}

void Input::update () {

    int i;
    sreg_data_t sreg_data;
    uint8_t button_data;
    uint8_t encoder_data;
    uint16_t adc_data;
    uint16_t *pot_data_p = (uint16_t*) &pot_data;

    // read buttons and encoder
    // note that the encoder gives a zero value when moving
    sreg_data = _readShiftRegister();
    button_state = sreg_data.button_data;
    if (sreg_data.encoder_data & 0x08) {osc_waveform = SINE;}
    else if (sreg_data.encoder_data & 0x04) {osc_waveform = CHORD;}
    else if (sreg_data.encoder_data & 0x01) {osc_waveform = CAPACITOR;}
    else if (sreg_data.encoder_data & 0x02) {osc_waveform = SAW_UP;}
    else if (sreg_data.encoder_data & 0x80) {osc_waveform = SQUARE;}
    else if (sreg_data.encoder_data & 0x10) {osc_waveform = PULSE;}
    else if (sreg_data.encoder_data & 0x40) {osc_waveform = LASER_SAW;}
    else if (sreg_data.encoder_data & 0x20) {osc_waveform = LASER_SQUARE;}

    // read the pots
    for (i = 0; i < N_POTS; i++) {
        digitalWrite(PIN_MUX_S0, POT_ROUTING[i].mux_setting & 0x1);
        digitalWrite(PIN_MUX_S1, POT_ROUTING[i].mux_setting & 0x2);
        digitalWrite(PIN_MUX_S2, POT_ROUTING[i].mux_setting & 0x4);
        adc_data = _readAdc(POT_ROUTING[i].adc_channel);
        if (abs(*(pot_data_p + i) - adc_data) > POT_DEAD_ZONE) {
            *(pot_data_p + i) = adc_data;
        }
    }
}

sreg_data_t Input::_readShiftRegister () {

    int i;
    sreg_data_t data;
    uint16_t *data_p = (uint16_t*) &data;

    digitalWrite(PIN_SREG_LATCH, LOW);
    digitalWrite(PIN_SREG_LATCH, HIGH);

    *data_p = digitalRead(PIN_SREG_DATA);

    for (i = 1; i < 16; i++) {
        digitalWrite(PIN_SREG_CLK, HIGH);
        digitalWrite(PIN_SREG_CLK, LOW);
        *data_p = (*data_p << 1) + digitalRead(PIN_SREG_DATA);
    }

    // Serial.println(*data_p, HEX);

    return data;
}

void Input::printPots () {
    char stringBuffer[100];

    sprintf(stringBuffer, "osc_frequency = %d", pot_data.osc_frequency);
    Serial.println(stringBuffer);
    sprintf(stringBuffer, "lfo_depth     = %d", pot_data.lfo_depth);
    Serial.println(stringBuffer);
    sprintf(stringBuffer, "lfo_frequency = %d", pot_data.lfo_frequency);
    Serial.println(stringBuffer);
    sprintf(stringBuffer, "release_time  = %d", pot_data.release_time);
    Serial.println(stringBuffer);
    sprintf(stringBuffer, "filter_cutoff = %d", pot_data.filter_cutoff);
    Serial.println(stringBuffer);
}

uint16_t Input::_readAdc (int channel) {

    ADC->INPUTCTRL.bit.MUXPOS = channel;         // ADC+ to channel ANx
    ADC->SWTRIG.bit.START = 1;
    while (!ADC->INTFLAG.bit.RESRDY);
    return ADC->RESULT.reg;
}

void Input::_setupAdc () {

    GCLK->GENDIV.bit.ID = GCLK_ADC;             // select generator
    GCLK->GENDIV.bit.DIV = 8;                   // clock divider 1
    GCLK->GENCTRL.bit.ID = GCLK_ADC;            // select generator
    GCLK->GENCTRL.bit.SRC = 7;                  // FDPLL48M
    GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    GCLK->CLKCTRL.bit.ID = 0x1e;                // select clock GCLK_ADC
    GCLK->CLKCTRL.bit.GEN = GCLK_ADC;           // select generator
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    PORT->Group[0].PINCFG[5].bit.PMUXEN = 1;    // mux ADC on PA05 / pin A4
    PORT->Group[0].PINCFG[6].bit.PMUXEN = 1;    // mux ADC on PA06 / pin A5
    PORT->Group[0].PMUX[2].bit.PMUXO = 1;       // select AN05 (group B) for PA05
    PORT->Group[0].PMUX[3].bit.PMUXE = 1;       // select AN06 (group B) for PA06

    ADC->CTRLA.bit.ENABLE = 0;                  // disable peripheral before starting clock
    PM->APBCMASK.bit.ADC_ = 1;                  // start APBC clock

    ADC->CTRLB.bit.PRESCALER = 0;               // 4
    ADC->CTRLB.bit.RESSEL = 1;                  // 16 bit (averaging mode)
    ADC->INPUTCTRL.bit.MUXNEG = 0x18;           // ADC- to internal ground
    ADC->AVGCTRL.bit.SAMPLENUM = 6;             // average 64 (12 bit) samples with 2 automatic shifts gives a 16 bit result
    ADC->AVGCTRL.bit.ADJRES = 6;                // divide by 64 to get a 10 bit result
    ADC->SAMPCTRL.bit.SAMPLEN = 0;              // sample for one ADC clock cycle

    ADC->CTRLA.bit.ENABLE = 1;
}
