

void setupSpi () {

    GCLK->CLKCTRL.bit.ID = 0x15;                // select clock GCLK_SERCOM1_CORE
    GCLK->CLKCTRL.bit.GEN = 5;                  // clock generator 5
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    PORT->Group[0].PINCFG[16].bit.PMUXEN = 1;   // mux SPI MOSI on PA16 / pin 8
    PORT->Group[0].PINCFG[17].bit.PMUXEN = 1;   // mux SPI SCK on PA17 / pin 8
    PORT->Group[0].PINCFG[19].bit.PMUXEN = 1;   // mux SPI MISO on PA19 / pin 8
    PORT->Group[0].PMUX[8].bit.PMUXE = 1;       // select SERCOM1 PAD0 (group C) for PA16
    PORT->Group[0].PMUX[8].bit.PMUXO = 1;       // select SERCOM1 PAD1 (group C) for PA17
    PORT->Group[0].PMUX[9].bit.PMUXO = 1;       // select SERCOM1 PAD3 (group C) for PA19

    PM->APBCMASK.bit.SERCOM1_ = 1;              // enable pheripheral clock
    SERCOM1->SPI.CTRLA.bit.ENABLE = 0;

    SERCOM1->SPI.CTRLA.bit.MODE = 3;            // SPI master mode
    SERCOM1->SPI.CTRLB.bit.RXEN = 0;            // enable receiving

    digitalWrite(PIN_SPI_SS, HIGH);
    SERCOM1->SPI.CTRLA.bit.ENABLE = 1;
}
