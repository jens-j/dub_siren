#include "Arduino.h"
#include "dubsiren.h"

#ifndef __SPI_DMA_H
#define __SPI_DMA_H


#define DMA_CH_WRITE        0
#define DMA_CH_READ         1

#define SPI_CMD_READ        3
#define SPI_CMD_WRITE       2

enum dma_state_t {DMA_IDLE, DMA_READ, DMA_WRITE, DMA_WRITE_A, DMA_WRITE_B, DMA_READ_A, DMA_READ_B};

typedef struct __attribute__((packed)) spi_address_s {
    uint8_t byte2;
    uint8_t byte1;
    uint8_t byte0;
} spi_address_t;

typedef struct __attribute__((packed)) spi_buffer_s {
    uint8_t opcode;
    spi_address_t address;
    uint16_t data[SPI_BLOCK_SIZE];
} spi_buffer_t;


void printDataBuffer (uint16_t *buffer);

class SpiDma {
private:
    void _setupSpi ();
    void _setupDma ();
public:
    volatile spi_buffer_t read_buffer[2]; // the opcode and address are not used but the dma will return bytes when they are written
    volatile spi_buffer_t write_buffer[2]; // also, the compiler does not allow making these volatile
    volatile bool transfer_active;

    SpiDma ();
    void read (int buffer_index, uint32_t address);
    void write (int buffer_index, uint32_t address);
    void erase (); // erase the entire ram ic
    void irqHandler ();
    // void printReadBuffer (int index) {_printBuffer(&read_buffer[index].data[0]);}
    //void printWriteBuffer (int index) {_printBuffer(&write_buffer[index].data[0]);}
    void printReadBuffer (int index);
    void printWriteBuffer (int index);
    void printBtCount (int channel);
    void printDmaDescriptor (int channel, bool writeback);
};

#endif
