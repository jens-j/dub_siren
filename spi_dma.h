#include "Arduino.h"
#include "dubsiren.h"

#ifndef __SPI_DMA_H
#define __SPI_DMA_H


#define RAM_BYTES           (1 << 27)

#define SPI_DATA_SIZE       1024
#define SPI_DATA_BYTES      (SPI_DATA_SIZE << 1)
#define SPI_BUFFER_BYTES    (SPI_DATA_BYTES + 5)

#define DMA_CH_WRITE        0
#define DMA_CH_READ         1

#define SPI_CMD_READ        3
#define SPI_CMD_WRITE       2


typedef struct spi_buffer_s {
    uint8_t opcode;
    uint32_t address;
    uint16_t data[SPI_DATA_SIZE];
} spi_buffer_t;


class SpiDma {
private:

    volatile bool _transfer_active;

    void _setupSpi ();
    void _setupDma ();
public:
    spi_buffer_t read_buffer[2];
    spi_buffer_t write_buffer[2];

    SpiDma ();
    void read (int buffer_index);
    void write (int buffer_index);
    void erase (); // erase the entire ram ic
    void irqHandler ();
};

#endif
