#ifndef GYRO_SPI_H
#define GYRO_SPI_H

#include "stm32f7xx_hal.h"

#define GYRO_SPI_OK          0U
#define GYRO_SPI_ERROR       1U
#define GYRO_SPI_BAD_PARAM   2U

/** @brief 使用默认 SPI Mode 3 参数初始化陀螺仪 SPI1。 */
uint8_t gyro_spi_init(void);

/** @brief 按指定极性、相位和分频参数配置陀螺仪 SPI1。 */
uint8_t gyro_spi_configure(uint32_t polarity,
                           uint32_t phase,
                           uint32_t prescaler);

/** @brief 拉低陀螺仪片选。 */
void gyro_spi_cs_low(void);

/** @brief 拉高陀螺仪片选。 */
void gyro_spi_cs_high(void);

/** @brief 阻塞式收发一段 SPI 数据，并自动控制片选。 */
uint8_t gyro_spi_transfer(const uint8_t *tx_data,
                          uint8_t *rx_data,
                          uint16_t length);

#endif
