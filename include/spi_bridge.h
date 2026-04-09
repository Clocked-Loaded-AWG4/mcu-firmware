// spi_bridge.h
#ifndef SPI_BRIDGE_H
#define SPI_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    SPI_CH_A = 0,
    SPI_CH_B = 1
} spi_channel_t;

void spi_bridge_init(void);

void spi_bridge_init_channel(
    spi_channel_t ch,
    double initial_frequency_hz
);

void spi_bridge_set_waveform(
    spi_channel_t ch,
    const uint16_t *samples,
    uint16_t sample_count
);

void spi_bridge_set_frequency(
    spi_channel_t ch,
    double frequency_hz
);

void spi_bridge_process(void);

// void spi_bridge_process_frequency(
//     spi_channel_t ch
// );

const char* spi_bridge_get_last_error(void);

void spi_bridge_clear_error(void);

#endif // SPI_BRIDGE_H