#include "dubsiren.h"

#define SPI_DATA_SIZE   16
#define SPI_PACKET_SIZE (SPI_DATA_SIZE + 5)
#define SPI_CMD_READ    3
#define SPI_CMD_WRITE   2

__attribute__((aligned(16))) DmacDescriptor writeback_section[2];
__attribute__((aligned(16))) DmacDescriptor descriptor_section[2];

uint32_t loop_t0 = 0;
bool led_state = true;
volatile bool active = false;
uint8_t spi_send_buffer_read[SPI_PACKET_SIZE];
uint8_t spi_send_buffer_write[SPI_PACKET_SIZE];
uint8_t spi_recv_buffer[SPI_PACKET_SIZE];


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
    dma_descriptor.BTCNT.reg = SPI_PACKET_SIZE;
    dma_descriptor.SRCADDR.reg = (uint32_t) &spi_send_buffer_write + SPI_PACKET_SIZE;
    dma_descriptor.DSTADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    memcpy(&descriptor_section[0], &dma_descriptor, sizeof(DmacDescriptor));

    // setup spi -> sram dma channel
    DMAC->CHID.reg = 1;
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(3); // trigger on sercom1_rx
    dma_descriptor.BTCTRL.reg =
        DMAC_BTCTRL_VALID | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_CHCTRLB_LVL_LVL1_Val;
    dma_descriptor.BTCNT.reg = SPI_PACKET_SIZE;
    dma_descriptor.SRCADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    dma_descriptor.DSTADDR.reg = (uint32_t) &spi_recv_buffer + SPI_PACKET_SIZE;
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

// raise the SPI_SS pin after the transaction is completed
void DMAC_Handler () {
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    digitalWrite(PIN_SPI_SS, HIGH);
    DMAC->CHID.reg = 0;
    DMAC->CHINTFLAG.reg = 0xFF;
    active = false;
}


void printRecvBuffer () {

    int i;
    char buffer[100];

    for (i = 0; i < SPI_PACKET_SIZE; i++) {
        sprintf(buffer, "%02X", *(spi_recv_buffer + i));
        Serial.print(buffer);
    }
    Serial.println("");
}


void setup () {
    int i;
    char buffer[100];

    // initialize write buffers
    spi_send_buffer_read[0] = SPI_CMD_READ;
    spi_send_buffer_write[0] = SPI_CMD_WRITE;
    *((uint32_t*) &spi_send_buffer_read[1]) = 0x00050505;
    *((uint32_t*) &spi_send_buffer_write[1]) = 0x00050505;
    for (i = 0; i < SPI_DATA_SIZE; i++) {
        spi_send_buffer_read[5 + i] = 0;
        spi_send_buffer_write[5 + i] = i;
    }
    for (i = 0; i < SPI_PACKET_SIZE; i++) {
        spi_recv_buffer[i] = 0xff;
    }

    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SPI_CLK, OUTPUT);
    pinMode(PIN_SPI_MOSI, OUTPUT);
    pinMode(PIN_SPI_SS, OUTPUT);

    Serial.begin(115200);
    while (!Serial); // wait for a serial connection (terminal)

    setupSpi();
    setupDma();

    printRecvBuffer();
}


void loop () {

    int i;

    // write
    while (active);
    active = true;
    SERCOM1->SPI.CTRLB.reg = 0; // disable receiving
    descriptor_section[0].SRCADDR.reg = (uint32_t) &spi_send_buffer_write + SPI_PACKET_SIZE;
    digitalWrite(PIN_SPI_SS, LOW);
    DMAC->CHID.reg = 0;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

    // read
    while (active);
    active = true;
    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // enable receiving
    descriptor_section[0].SRCADDR.reg = (uint32_t) &spi_send_buffer_read + SPI_PACKET_SIZE;
    digitalWrite(PIN_SPI_SS, LOW);
    DMAC->CHID.reg = 1;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
    DMAC->CHID.reg = 0;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

    // delay(10);
    while (active);
    DMAC->CHID.reg = 1;
    Serial.println(DMAC->CHSTATUS.reg, HEX);
    Serial.println(DMAC->CHINTFLAG.reg, HEX);
    // Serial.println(SERCOM1->SPI.DATA.reg, HEX);
    // Serial.println(SERCOM1->SPI.STATUS.reg, HEX);
    // Serial.println(SERCOM1->SPI.INTFLAG.reg, HEX);

    printRecvBuffer();

    // blink led
    digitalWrite(PIN_LED, led_state);
    led_state = !led_state;

    delay(500);
}
