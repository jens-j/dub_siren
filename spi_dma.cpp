#include "Arduino.h"
#include "dubsiren.h"
#include "spi_dma.h"

// These are the records the DMAC uses
__attribute__((aligned(16))) DmacDescriptor _writeback_section[2];
__attribute__((aligned(16))) DmacDescriptor _descriptor_section[2];

//
DmacDescriptor dma_read_descriptor;
DmacDescriptor dma_write_descriptor;


SpiDma::SpiDma () {
    transfer_active = false;

    _setupSpi();
    _setupDma();
    erase();
}


// the write dma channel is used to issue a read command and to generate the clock for the SPI read sequence
void SpiDma::read (int index, uint32_t address) {

    while (transfer_active);
    transfer_active = true;

    write_buffer[index].opcode = SPI_CMD_READ; // set packet opcode
    *((uint32_t*) &write_buffer[index].address) = address; // set packet address

    // _descriptor_section[DMA_CH_WRITE].SRCADDR.reg = // set dma write descriptor source address
    //     (uint32_t) &write_buffer[index] + SPI_BUFFER_BYTES;
    // _descriptor_section[DMA_CH_READ].DSTADDR.reg = // set dma read descriptor destination address
    //     (uint32_t) &read_buffer[index] + SPI_BUFFER_BYTES;

    dma_write_descriptor.SRCADDR.reg = (uint32_t) &write_buffer[index] + SPI_BUFFER_BYTES;
    dma_read_descriptor.DSTADDR.reg = (uint32_t) &read_buffer[index] + SPI_BUFFER_BYTES;

    memcpy(&_descriptor_section[DMA_CH_WRITE], &dma_write_descriptor, sizeof(DmacDescriptor));
    memcpy(&_descriptor_section[DMA_CH_READ], &dma_read_descriptor, sizeof(DmacDescriptor));

    digitalWrite(PIN_SPI_SS, LOW); // pull down SS to initiate a transfer

    SERCOM1->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN; // enable SPI receiving
    DMAC->CHID.reg = DMA_CH_READ;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable dma read channel
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable dma write channel
}


void SpiDma::write (int index, uint32_t address) {

    while (transfer_active);
    transfer_active = true;

    write_buffer[index].opcode = SPI_CMD_WRITE; // set packet opcode
    *((uint32_t*) &write_buffer[index].address) = address; // set packet address

    _descriptor_section[DMA_CH_WRITE].SRCADDR.reg = // set dma descriptor source address
        (uint32_t) &write_buffer[index] + SPI_BUFFER_BYTES;

    digitalWrite(PIN_SPI_SS, LOW); // pull down SS to initiate a transfer

    SERCOM1->SPI.CTRLB.reg = 0; // disable SPI receiving
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable the DMA channel
}


void SpiDma::erase () {

    int i;

    while (transfer_active);
    transfer_active = true;

    SERCOM1->SPI.CTRLB.reg = 0; // disable receiving
    digitalWrite(PIN_SPI_SS, LOW);

    // write opcode
    while (!SERCOM1->SPI.INTFLAG.bit.DRE);
    SERCOM1->SPI.DATA.reg = SPI_CMD_WRITE;

    // write zero address and 2**27 zeros
    for (i = 0; i < 1; i++) {
        while (!SERCOM1->SPI.INTFLAG.bit.DRE);
        SERCOM1->SPI.DATA.reg = 0;
    }

    digitalWrite(PIN_SPI_SS, HIGH);
    transfer_active = false;
}


// raise slave select when the tx is done
void SpiDma::irqHandler () {
    while (!SERCOM1->SPI.INTFLAG.bit.TXC);
    digitalWrite(PIN_SPI_SS, HIGH);
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL; // 0xFF;
    transfer_active = false;
}


void SpiDma::printWriteBuffer (int index) {

    int i;
    char line_buffer[40];

    Serial.println("Write Buffer:");
    sprintf(line_buffer, "opcode  = 0x%02X", write_buffer[index].opcode);
    Serial.println(line_buffer);
    sprintf(line_buffer, "address = 0x%08X", write_buffer[index].address);
    Serial.println(line_buffer);
    for (i = 0; i < 8; i++) {
        sprintf(line_buffer, "%04X ", write_buffer[index].data[i]);
        Serial.print(line_buffer);
    }
    Serial.print("... ");
    for (i = SPI_BLOCK_SIZE - 8; i < SPI_BLOCK_SIZE; i++) {
        sprintf(line_buffer, "%04X ", write_buffer[index].data[i]);
        Serial.print(line_buffer);
    }
    Serial.println("");
}


void SpiDma::printReadBuffer (int index) {

    int i;
    char line_buffer[40];

    Serial.println("Read Buffer:");
    for (i = 0; i < 8; i++) {
        sprintf(line_buffer, "%04X ", read_buffer[index].data[i]);
        Serial.print(line_buffer);
    }
    Serial.print("... ");
    for (i = SPI_BLOCK_SIZE - 8; i < SPI_BLOCK_SIZE; i++) {
        sprintf(line_buffer, "%04X ", read_buffer[index].data[i]);
        Serial.print(line_buffer);
    }
    Serial.println("");
}


void SpiDma::printBtCount (int channel) {
    Serial.println(_writeback_section[channel].DSTADDR.reg, HEX);
}


void SpiDma::printDmaDescriptor (int channel, bool writeback) {

    char line_buffer[40];
    DmacDescriptor *desc;

    if (writeback) {
        desc = &_writeback_section[channel];
        sprintf(line_buffer, "Writeback Section[%d]", channel);
    } else {
        desc = &_descriptor_section[channel];
        sprintf(line_buffer, "Descriptor Section Section[%d]", channel);
    }
    Serial.println(line_buffer);
    sprintf(line_buffer, "BTCTRL = %04X", desc->BTCTRL.reg);
    Serial.println(line_buffer);
    sprintf(line_buffer, "BTCNT = %04X", desc->BTCNT.reg);
    Serial.println(line_buffer);
    sprintf(line_buffer, "SRCADDR = %08X", desc->SRCADDR.reg);
    Serial.println(line_buffer);
    sprintf(line_buffer, "DSTADDR = %08X", desc->DSTADDR.reg);
    Serial.println(line_buffer);
    sprintf(line_buffer, "DESCADDR = %08X", desc->DESCADDR.reg);
    Serial.println(line_buffer);
}


void printDataBuffer (uint16_t *buffer) {

    Serial.println((uint32_t) buffer, HEX);
    //Serial.println(*buffer, HEX);


}


void SpiDma::_setupSpi () {

    Serial.println("_setupSpi");

    GCLK->GENDIV.bit.ID = GLCK_SPI;             // select generator
    GCLK->GENDIV.bit.DIV = 1;                   //
    GCLK->GENCTRL.bit.ID = GLCK_SPI;            // select generator
    GCLK->GENCTRL.bit.SRC = 7;                  //
    GCLK->GENCTRL.bit.IDC = 1;                  // improve duty cycle
    GCLK->GENCTRL.bit.GENEN = 1;                // enable generator
    GCLK->CLKCTRL.bit.ID = 0x15;                // select clock GCLK_SERCOM1_CORE
    GCLK->CLKCTRL.bit.GEN = GLCK_SPI;           // clock generator 5
    GCLK->CLKCTRL.bit.CLKEN = 1;                // enable

    PORT->Group[0].PINCFG[16].reg |= PORT_PINCFG_PMUXEN;  // mux SPI MOSI on PA16 / pin 8
    PORT->Group[0].PINCFG[17].reg |= PORT_PINCFG_PMUXEN;  // mux SPI SCK  on PA17 / pin 9
    PORT->Group[0].PINCFG[19].reg |= PORT_PINCFG_PMUXEN;  // mux SPI MISO on PA19 / pin 10
    PORT->Group[0].PMUX[8].reg = PORT_PMUX_PMUXO_C | PORT_PMUX_PMUXE_C;
    PORT->Group[0].PMUX[9].reg = PORT_PMUX_PMUXO_C;

    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;    // enable pheripheral clock
    SERCOM1->SPI.CTRLA.bit.ENABLE = 0;

    SERCOM1->SPI.BAUD.reg = 5;                  // ~4.8 MHz, faster causes DMA problems
    SERCOM1->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER | SERCOM_SPI_CTRLA_DIPO(3);

    digitalWrite(PIN_SPI_SS, HIGH);
    SERCOM1->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
}


void SpiDma::_setupDma () {

    DmacDescriptor dma_descriptor;

    Serial.println("_setupDma");

    PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC;

    DMAC->CTRL.bit.DMAENABLE = 0;

    DMAC->BASEADDR.reg = (uint32_t) &_descriptor_section;
	DMAC->WRBADDR.reg = (uint32_t) &_writeback_section;
    DMAC->QOSCTRL.reg =
        DMAC_QOSCTRL_DQOS_MEDIUM | DMAC_QOSCTRL_FQOS_HIGH | DMAC_QOSCTRL_WRBQOS_MEDIUM;

    // setup sram -> spi dma channel
    DMAC->CHID.reg = DMA_CH_WRITE;
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(4); // trigger on sercom1_tx
    DMAC->CHINTENSET.bit.TCMPL = 1; // enable TX ready interrupt
    dma_write_descriptor.DESCADDR.reg = 0;
    dma_write_descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC;
    dma_write_descriptor.BTCNT.reg = SPI_BUFFER_BYTES;
    dma_write_descriptor.SRCADDR.reg = (uint32_t) &write_buffer[0] + SPI_BUFFER_BYTES;
    dma_write_descriptor.DSTADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    memcpy(&_descriptor_section[DMA_CH_WRITE], &dma_write_descriptor, sizeof(DmacDescriptor));

    // setup spi -> sram dma channel
    DMAC->CHID.reg = DMA_CH_READ;
    DMAC->CHCTRLB.reg =
        DMAC_CHCTRLB_TRIGACT_BEAT | DMAC_CHCTRLB_TRIGSRC(3) | DMAC_CHCTRLB_LVL_LVL1_Val; // trigger on sercom1_rx
    dma_read_descriptor.DESCADDR.reg = 0;
    dma_read_descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST;
    dma_read_descriptor.BTCNT.reg = SPI_BUFFER_BYTES;
    dma_read_descriptor.SRCADDR.reg = (uint32_t) &SERCOM1->SPI.DATA.reg;
    dma_read_descriptor.DSTADDR.reg = (uint32_t) &read_buffer[0] + SPI_BUFFER_BYTES;
    memcpy(&_descriptor_section[DMA_CH_READ], &dma_read_descriptor, sizeof(DmacDescriptor));

    // enable SPI TX ready interrupt
    NVIC_ClearPendingIRQ(DMAC_IRQn);
    NVIC_SetPriority(DMAC_IRQn, 2);
    NVIC_EnableIRQ(DMAC_IRQn);

    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1;
}
