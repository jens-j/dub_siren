#include <sam.h>
#include "Arduino.h"
#include "dubsiren.h"

#ifndef __TIMER_H
#define __TIMER_H

// configure TCC0 timer for LED pwm
void setupTimer () {

    int i;

    // setup the clock for TCC0
    // GCLK->GENDIV.bit.ID = GLCK_TCC;             // select generator
    // GCLK->GENDIV.bit.DIV = 1;                   // clock divider
    //
    // GCLK->GENCTRL.bit.ID = GLCK_TCC;            // select generator
    // GCLK->GENCTRL.bit.SRC = 7;                  // FDPLL48M TODO: use the PLL to get 48 kHz
    // GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    // GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    //
    // GCLK->CLKCTRL.bit.ID = 0x1A;                // select clock GCLK_TCC0 (and GCLK_TCC1)
    // GCLK->CLKCTRL.bit.GEN = GLCK_TCC;           // clock generator
    // GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |              // Improve duty cycle
                        GCLK_GENCTRL_GENEN |            // Enable generic clock gen
                        GCLK_GENCTRL_SRC_DFLL48M |      // Select 48MHz as source
                        GCLK_GENCTRL_ID(GLCK_TCC);      // Select GLCK_TCC

    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |             // Divide 48 MHz by 1
                       GCLK_GENDIV_ID(GLCK_TCC);        // Apply to GLCK_TCC

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |            // Enable generic clock
                        GCLK_CLKCTRL_GEN(GLCK_TCC) |    // Select GLCK_TCC
                        GCLK_CLKCTRL_ID_TCC0_TCC1;      // Feed GCLK4 to TCC0/1

    // enable peripheral mux
    PORT->Group[0].PINCFG[20].bit.PMUXEN = 1;

    // set port mux values
    PORT->Group[0].PMUX[10].bit.PMUXE = PORT_PMUX_PMUXE_F;      // PA20 -> TCC0/WO[6] (F)

    // start APBC clock
    PM->APBCMASK.bit.TCC0_ = 1;

    // setup TCC0
    TCC0->CTRLA.bit.ENABLE = 0;                 // disable TCC
    TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
    TCC0->PER.bit.PER = 0xFFFF;
    TCC0->WEXCTRL.reg = TCC_WEXCTRL_OTMX(2);    // put CC0 on all 8 output channels
    TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
}

#endif
