#include "Arduino.h"
#include "dubsiren.h"

#ifndef __SPI_DMA_H
#define __SPI_DMA_H


#define RAM_BYTES           (1 << 17)
#define RAM_SAMPLES         (RAM_BYTES >> 1)
#define RAM_BLOCKS          (RAM_BYTES / SPI_BLOCK_SIZE)

#define SPI_BLOCK_SIZE      1024
#define SPI_BLOCK_BYTES     (SPI_BLOCK_SIZE << 1)
#define SPI_BUFFER_BYTES    (SPI_BLOCK_BYTES + 5)

#define DMA_CH_WRITE        0
#define DMA_CH_READ         1

#define SPI_CMD_READ        3
#define SPI_CMD_WRITE       2


typedef struct __attribute__((packed)) spi_buffer_s {
    uint8_t opcode;
    uint32_t address;
    uint16_t data[SPI_BLOCK_SIZE];
} spi_buffer_t;


class SpiDma {
private:
    void _setupSpi ();
    void _setupDma ();
    void _printBuffer (uint16_t *buffer);
public:
    spi_buffer_t read_buffer[2]; // the opcode and address are not used but the dma will return bytes when they are written
    spi_buffer_t write_buffer[2]; // also, the compiler does not allow making these volatile
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
};

#endif
