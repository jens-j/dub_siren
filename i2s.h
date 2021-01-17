#include "Arduino.h"
#include "dubsiren.h"

#ifndef __I2S_H
#define __I2S_H

void setupI2S() {
    // set the I2S module to 48 kHz tx 16-bit mono mode

    // setup the clock for the I2S peripheral
    GCLK->GENDIV.bit.ID = GLCK_I2S;             // select generator
    GCLK->GENDIV.bit.DIV = 31;                  // clock divider 48 MHz / 31 / 16 / 2 = 48.387 kHz
    GCLK->GENCTRL.bit.ID = GLCK_I2S;            // select generator
    GCLK->GENCTRL.bit.SRC = 7;                  // FDPLL48M TODO: use the PLL to get 48 kHz
    GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    GCLK->CLKCTRL.bit.ID = 0x23;                // select clock GCLK_I2S_0
    GCLK->CLKCTRL.bit.GEN = GLCK_I2S;           // clock generator 
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

#endif
