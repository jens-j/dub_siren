#include "Arduino.h"
#include "dubsiren.h"
#include "fixedpoint.h"
#include "input.h"


Input::Input () {

    _setupAdc();
    update();
}

void Input::update () {

    int new_osc_frequency = this->_readAdc(ADC_CH_OSC);
    if (abs(osc_frequency - new_osc_frequency) > POT_DEAD_ZONE) {
        osc_frequency = new_osc_frequency;
    }

    int new_lfo_waveform = this->_readAdc(ADC_CH_SHAPE);
    if (abs(lfo_waveform - new_lfo_waveform) > POT_DEAD_ZONE) {
        lfo_waveform = new_lfo_waveform;
    }

    int new_lfo_frequency = this->_readAdc(ADC_CH_LFO);
    if (abs(lfo_frequency - new_lfo_frequency) > POT_DEAD_ZONE) {
        lfo_frequency = new_lfo_frequency;
    }

    int new_lfo_depth = this->_readAdc(ADC_CH_DEPTH);
    if (abs(lfo_depth - new_lfo_depth) > POT_DEAD_ZONE) {
        lfo_depth = new_lfo_depth;
    }

    int new_filter_cutoff = this->_readAdc(ADC_CH_FILTER);
    if (abs(filter_cutoff - new_filter_cutoff) > POT_DEAD_ZONE) {
        filter_cutoff = new_filter_cutoff;
    }

    int new_decay_time = this->_readAdc(ADC_CH_DECAY);
    if (abs(decay_time - new_decay_time) > POT_DEAD_ZONE) {
        decay_time = new_decay_time;
    }
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

    PORT->Group[1].PINCFG[2].bit.PMUXEN = 1;    // mux ADC on PB02 / pin A1
    PORT->Group[1].PINCFG[3].bit.PMUXEN = 1;    // mux ADC on PB03 / pin A2
    PORT->Group[0].PINCFG[4].bit.PMUXEN = 1;    // mux ADC on PA04 / pin A3
    PORT->Group[0].PINCFG[5].bit.PMUXEN = 1;    // mux ADC on PA05 / pin A4
    PORT->Group[0].PINCFG[6].bit.PMUXEN = 1;    // mux ADC on PA06 / pin A5
    PORT->Group[0].PINCFG[3].bit.PMUXEN = 1;    // mux VREFA on PA03 / pin AREF
    PORT->Group[1].PMUX[1].bit.PMUXE = 1;       // select AN10 (group B) for PB02
    PORT->Group[1].PMUX[1].bit.PMUXO = 1;       // select AN11 (group B) for PB03
    PORT->Group[0].PMUX[2].bit.PMUXE = 1;       // select AN04 (group B) for PA04
    PORT->Group[0].PMUX[2].bit.PMUXO = 1;       // select AN05 (group B) for PA05
    PORT->Group[0].PMUX[3].bit.PMUXE = 1;       // select AN06 (group B) for PA06
    PORT->Group[0].PMUX[1].bit.PMUXO = 1;       // select VREFA (group B) for PA03

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
