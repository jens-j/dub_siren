#include "Arduino.h"
#include "dubsiren.h"
#include "spi_dma.h"


__attribute__((aligned(16))) DmacDescriptor _writeback_section[2];
__attribute__((aligned(16))) DmacDescriptor _descriptor_section[2];

const uint8_t ZERO = 0;


SpiDma::SpiDma () {
    _transfer_active = false;
    read_buffer[0].opcode = SPI_CMD_READ;
    read_buffer[1].opcode = SPI_CMD_READ;
    write_buffer[0].opcode = SPI_CMD_WRITE;
    write_buffer[1].opcode = SPI_CMD_WRITE;

    _setupSpi();
    _setupDma();
    erase();
}


void SpiDma::read (int index) {

    while (_transfer_active);
    _transfer_active = true;

    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // enable receiving
    _descriptor_section[DMA_CH_READ].DSTADDR.reg =
        (uint32_t) &read_buffer[index] + SPI_BUFFER_BYTES;
    digitalWrite(PIN_SPI_SS, LOW);
    DMAC->CHID.reg = DMA_CH_READ;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}


void SpiDma::write (int index) {

    while (_transfer_active);
    _transfer_active = true;

    SERCOM1->SPI.CTRLB.reg = 0; // disable receiving
    _descriptor_section[DMA_CH_WRITE].DSTADDR.reg =
        (uint32_t) &write_buffer[index] + SPI_BUFFER_BYTES;
    digitalWrite(PIN_SPI_SS, LOW);
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}


void SpiDma::erase () {

    int i;

    while (_transfer_active);
    _transfer_active = true;

    SERCOM1->SPI.CTRLB.reg = 0; // disable receiving
    digitalWrite(PIN_SPI_SS, LOW);

    // write opcode
    while (!SERCOM1->SPI.INTFLAG.bit.DRE);
    SERCOM1->SPI.DATA.reg = SPI_CMD_WRITE;

    // // write zero address and 2**27 zeros
    // for (i = 0; i < 1; i++) {
    //     while (!SERCOM1->SPI.INTFLAG.bit.DRE);
    //     SERCOM1->SPI.DATA.reg = 0;
    // }

    digitalWrite(PIN_SPI_SS, HIGH);
    _transfer_active = false;
}


// raise slave select when the tx is done
void SpiDma::irqHandler () {
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    digitalWrite(PIN_SPI_SS, HIGH);
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHINTFLAG.reg = 0xFF;
    _transfer_active = false;
}


void SpiDma::_setupSpi () {

    // Why does this work without configuring GLCK_SPI?
    GCLK->CLKCTRL.bit.ID = 0x15;                // select clock GCLK_SERCOM1_CORE
    GCLK->CLKCTRL.bit.GEN = GLCK_SPI;           // clock generator 5
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


void SpiDma::_setupDma () {

    DmacDescriptor dma_descriptor;

    dma_descriptor.DESCADDR.reg = 0;

    PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC;

    DMAC->CTRL.bit.DMAENABLE = 0;

    DMAC->BASEADDR.reg = (uint32_t) &_descriptor_section;
	DMAC->WRBADDR.reg = (uint32_t) &_writeback_section;

    // setup sram -> spi dma channel
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(4); // trigger on sercom1_tx
    DMAC->CHINTENSET.bit.TCMPL = 1; // enable TX ready interrupt
    dma_descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC;
    dma_descriptor.BTCNT.reg = SPI_BUFFER_BYTES;
    dma_descriptor.SRCADDR.reg = (uint32_t) &write_buffer[0] + SPI_BUFFER_BYTES;
    dma_descriptor.DSTADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    memcpy(&_descriptor_section[DMA_CH_WRITE], &dma_descriptor, sizeof(DmacDescriptor));

    // setup spi -> sram dma channel
    DMAC->CHID.reg = DMA_CH_READ;
    DMAC->CHCTRLB.reg =
        DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(3) | DMAC_CHCTRLB_LVL_LVL1_Val; // trigger on sercom1_rx
    dma_descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST;
    dma_descriptor.BTCNT.reg = SPI_BUFFER_BYTES;
    dma_descriptor.SRCADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    dma_descriptor.DSTADDR.reg = (uint32_t) &read_buffer[0] + SPI_BUFFER_BYTES;
    memcpy(&_descriptor_section[DMA_CH_READ], &dma_descriptor, sizeof(DmacDescriptor));

    // enable SPI TX ready interrupt
    NVIC_ClearPendingIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, 2);
    NVIC_EnableIRQ(DMAC_IRQn);

    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1;
}
