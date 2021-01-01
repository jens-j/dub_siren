#include "dubsiren.h"

#define BLOCK_SIZE 8

__attribute__((aligned(16))) DmacDescriptor writeback_section[2];
__attribute__((aligned(16))) DmacDescriptor descriptor_section[2];

uint32_t led_t0 = 0;
bool led_state = true;
uint8_t spi_send_buffer[BLOCK_SIZE];
uint8_t spi_recv_buffer[BLOCK_SIZE];
volatile bool active = false;


inline void get_address_bytes (uint32_t address, uint8_t *addrbytes) {
    addrbytes[0] = (uint8_t) (address >> 16);
    addrbytes[1] = (uint8_t) (address >> 8);
    addrbytes[2] = (uint8_t) address;
}


inline void write (uint8_t *data, uint16_t length) {
    int i;

    for (i = 0; i < length; i++) {
        //Serial.println('a');
        while (!SERCOM1->SPI.INTFLAG.bit.DRE);
        SERCOM1->SPI.DATA.reg = *(data + i);
    }
}


inline void read (uint8_t *data, uint16_t length) {
    int i;

    for (i = 0; i < length; i++) {
        SERCOM1->SPI.INTENCLR.bit.RXC = 1;
        SERCOM1->SPI.DATA.reg = 0;
        while (!SERCOM1->SPI.INTFLAG.bit.RXC);
        *(data + i) = SERCOM1->SPI.DATA.reg;
    }
}


inline void read_from_address (uint32_t address, uint8_t *data, uint16_t length) {

    uint8_t addrbytes[3];
    get_address_bytes(address, addrbytes);

    digitalWrite(PIN_SPI_SS, LOW);
    write(&addrbytes[0], 3);
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    read(data, length);
    while (!SERCOM1->SPI.INTFLAG.bit.RXC);
    digitalWrite(PIN_SPI_SS, HIGH);
}


inline void write_to_address (uint32_t address, uint8_t *data, uint16_t length) {

    uint8_t addrbytes[3];
    get_address_bytes(address, addrbytes);

    digitalWrite(PIN_SPI_SS, LOW);
    write(&addrbytes[0], 3);
    write(data, length);
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    digitalWrite(PIN_SPI_SS, HIGH);
}


void setupDma () {

    DmacDescriptor dma_descriptor;

    dma_descriptor.DESCADDR.reg = 0;

    PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC;

    DMAC->CTRL.bit.DMAENABLE = 0;

    DMAC->BASEADDR.reg = (uint32_t) &descriptor_section;
	DMAC->WRBADDR.reg = (uint32_t) &writeback_section;

    // setup sram -> spi dma channel
    DMAC->CHID.reg = 0;
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(4); // trigger on sercom1_tx
    DMAC->CHINTENSET.bit.TCMPL = 1;
    dma_descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC;
    dma_descriptor.BTCNT.reg = BLOCK_SIZE;
    dma_descriptor.SRCADDR.reg = (uint32_t) &spi_send_buffer + BLOCK_SIZE;
    dma_descriptor.DSTADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    memcpy(&descriptor_section[0], &dma_descriptor, sizeof(DmacDescriptor));


    // setup spi -> sram dma channel
    DMAC->CHID.reg = 1;
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(3); // trigger on sercom1_rx
    dma_descriptor.BTCTRL.reg =
        DMAC_BTCTRL_VALID | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_CHCTRLB_LVL_LVL1_Val;
    dma_descriptor.BTCNT.reg = BLOCK_SIZE;
    dma_descriptor.SRCADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    dma_descriptor.DSTADDR.reg = (uint32_t) &spi_recv_buffer + BLOCK_SIZE;
    memcpy(&descriptor_section[1], &dma_descriptor, sizeof(DmacDescriptor));

    NVIC_ClearPendingIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, 1);
    NVIC_EnableIRQ(DMAC_IRQn);

    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1;
}


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


void DMAC_Handler() {
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    digitalWrite(PIN_SPI_SS, HIGH);
    DMAC->CHID.reg = 0;
    DMAC->CHINTFLAG.reg = 0xFF;
    active = false;
}


void setup () {
    int i;
    char buffer[100];

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SPI_CLK, OUTPUT);
    pinMode(PIN_SPI_MOSI, OUTPUT);
    pinMode(PIN_SPI_SS, OUTPUT);

    Serial.begin(115200);
    // while (!Serial); // wait for a serial connection (terminal)

    for (i = 0; i < BLOCK_SIZE; i++) {
        spi_send_buffer[i] = (uint8_t) i;
        spi_recv_buffer[i] = (uint8_t) i;
    }

    setupSpi();
    setupDma();

    for (i = 0; i < sizeof(DmacDescriptor); i++) {
        sprintf(buffer, "%02X", *(((uint8_t*) &descriptor_section) + i));
        Serial.print(buffer);
    }
    Serial.println("");

    for (i = 0; i < BLOCK_SIZE; i++) {
        sprintf(buffer, "%02X", *(spi_send_buffer + i));
        Serial.print(buffer);
    }
    Serial.println("");
}


void loop () {

    int i;
    char buffer[100];

    if (active == false) {
        delayMicroseconds(10);
        digitalWrite(PIN_SPI_SS, LOW);
        DMAC->CHID.reg = 1;
        DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
        DMAC->CHID.reg = 0;
        DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
        active = true;
    }

    // blink led at 2 Hz
    int t = millis();
    if (t - led_t0 > MAIN_LOOP_MS) {

        digitalWrite(PIN_LED, led_state);
        led_state = !led_state;
        led_t0 += MAIN_LOOP_MS;

        DMAC->CHID.reg = 1;
        Serial.println(DMAC->CHINTFLAG.reg, HEX);
        Serial.println(SERCOM1->SPI.INTFLAG.reg, HEX);
        for (i = 0; i < BLOCK_SIZE; i++) {
            sprintf(buffer, "%02X", *(spi_recv_buffer + i));
            Serial.print(buffer);
        }
        Serial.println("");
    }
}
