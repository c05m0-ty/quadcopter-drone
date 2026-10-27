#ifndef GYRO_SPI_H
#define GYRO_SPI_H

#include "stm32f7xx_hal.h"

#define GYRO_SPI_OK          0U
#define GYRO_SPI_ERROR       1U
#define GYRO_SPI_BAD_PARAM   2U

uint8_t gyro_spi_init(void);
uint8_t gyro_spi_configure(uint32_t polarity,
                           uint32_t phase,
                           uint32_t prescaler);
void gyro_spi_cs_low(void);
void gyro_spi_cs_high(void);

uint8_t gyro_spi_transfer(const uint8_t *tx_data,
                          uint8_t *rx_data,
                          uint16_t length);

#endif
