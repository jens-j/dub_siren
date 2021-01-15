#include "dubsiren.h"

char msg_write[9] = {2,0,5,5,5,1,2,3,4};
char msg_read[9]  = {3,0,5,5,5,0,0,0,0};
char recv_data[9];
char dump;

void setupSpi () {

    GCLK->CLKCTRL.bit.ID = 0x15;
    GCLK->CLKCTRL.bit.GEN = 3;
    GCLK->CLKCTRL.bit.CLKEN = 1;

    PORT->Group[0].PINCFG[16].reg |= PORT_PINCFG_PMUXEN;  // mux SPI MOSI on PA16 / pin 8
    PORT->Group[0].PINCFG[17].reg |= PORT_PINCFG_PMUXEN;  // mux SPI SCK  on PA17 / pin 9
    PORT->Group[0].PINCFG[19].reg |= PORT_PINCFG_PMUXEN;  // mux SPI MISO on PA19 / pin 10
    PORT->Group[0].PMUX[8].reg = PORT_PMUX_PMUXO_C | PORT_PMUX_PMUXE_C;
    PORT->Group[0].PMUX[9].reg = PORT_PMUX_PMUXO_C;

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1; // enable pheripheral clock
    SERCOM1->SPI.CTRLA.bit.ENABLE = 0;

    SERCOM1->SPI.BAUD.reg = 2;
    SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER | SERCOM_SPI_CTRLA_DIPO(3);
    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // enable receiving

    digitalWrite(PIN_SPI_SS, HIGH);
    SERCOM1->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
}


void setup () {

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SPI_CLK, OUTPUT);
    pinMode(PIN_SPI_MOSI, OUTPUT);
    pinMode(PIN_SPI_SS, OUTPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for a serial connection (terminal)

    setupSpi();
}


void loop () {

    int i;
    char buffer[100];

    SERCOM1->SPI.CTRLB.reg = 0; // disable receiving
    digitalWrite(PIN_SPI_SS, LOW);
    for (i = 0; i < 9; i++) {
        while (!SERCOM1->SPI.INTFLAG.bit.DRE);
        SERCOM1->SPI.DATA.reg = msg_write[i];
        // while (!SERCOM1->SPI.INTFLAG.bit.RXC);
        // dump = SERCOM1->SPI.DATA.reg;
    }
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    digitalWrite(PIN_SPI_SS, HIGH);

    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // enable receiving
    digitalWrite(PIN_SPI_SS, LOW);
    for (i = 0; i < 9; i++) {
        while (!SERCOM1->SPI.INTFLAG.bit.DRE);
        SERCOM1->SPI.DATA.reg = msg_read[i];
        while (!SERCOM1->SPI.INTFLAG.bit.RXC);
        recv_data[i] = SERCOM1->SPI.DATA.reg;
    }
    digitalWrite(PIN_SPI_SS, HIGH);

    for (i = 0; i < 9; i++) {
        sprintf(buffer, "%02X", *(recv_data + i));
        Serial.print(buffer);
    }
    Serial.println("");
    Serial.println(SERCOM1->SPI.STATUS.reg, HEX);
    Serial.println(SERCOM1->SPI.INTFLAG.reg, HEX);
    Serial.println("");

    delay(100);
}
