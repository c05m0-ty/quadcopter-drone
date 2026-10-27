#ifndef GYRO_DMA_H
#define GYRO_DMA_H

#include "stm32f7xx_hal.h"

#define GYRO_DMA_OK         0U
#define GYRO_DMA_BUSY       1U
#define GYRO_DMA_BAD_PARAM  2U
#define GYRO_DMA_ERROR      3U

uint8_t gyro_dma_init(SPI_HandleTypeDef *spi);
void gyro_dma_deinit(SPI_HandleTypeDef *spi);

uint8_t gyro_dma_start(const uint8_t *tx_data,
                       uint8_t *rx_data,
                       uint16_t length);
uint8_t gyro_dma_take_complete(void);
uint8_t gyro_dma_take_error(void);
uint8_t gyro_dma_is_busy(void);
uint8_t gyro_dma_abort(void);

#endif
