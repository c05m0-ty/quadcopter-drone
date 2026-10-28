#ifndef GYRO_DMA_H
#define GYRO_DMA_H

#include "stm32f7xx_hal.h"

#define GYRO_DMA_OK         0U
#define GYRO_DMA_BUSY       1U
#define GYRO_DMA_BAD_PARAM  2U
#define GYRO_DMA_ERROR      3U

/** @brief 初始化并关联 SPI1 使用的 DMA2 Stream0/3。 */
uint8_t gyro_dma_init(SPI_HandleTypeDef *spi);

/** @brief 停止并释放 SPI1 使用的 DMA 数据流。 */
void gyro_dma_deinit(SPI_HandleTypeDef *spi);

/** @brief 启动一次 SPI1 DMA 全双工传输。 */
uint8_t gyro_dma_start(const uint8_t *tx_data,
                       uint8_t *rx_data,
                       uint16_t length);

/** @brief 原子读取并清除 DMA 完成标志。 */
uint8_t gyro_dma_take_complete(void);

/** @brief 原子读取并清除 DMA 错误标志。 */
uint8_t gyro_dma_take_error(void);

/** @brief 查询 DMA 是否仍在传输。 */
uint8_t gyro_dma_is_busy(void);

/** @brief 中止当前 DMA 传输并释放 SPI 片选。 */
uint8_t gyro_dma_abort(void);

#endif
