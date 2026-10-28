#include "gyro_spi.h"
#include "gyro_dma.h"

#define GYRO_CS_GPIO_PORT    GPIOA
#define GYRO_CS_PIN          GPIO_PIN_4
#define GYRO_SPI_TIMEOUT_MS  10U

SPI_HandleTypeDef hgyro_spi;

/**
 * @brief 初始化 SPI1 的 GPIO、片选和 DMA 底层资源。
 * @param hspi HAL SPI 句柄；仅处理 SPI1。
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio = {0};

    if (hspi->Instance != SPI1)
    {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    gpio.Pin = GYRO_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &gpio);

    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    (void)gyro_dma_init(hspi);
}

/**
 * @brief 释放 SPI1 的 DMA、时钟和 GPIO 底层资源。
 * @param hspi HAL SPI 句柄；仅处理 SPI1。
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    gyro_dma_deinit(hspi);
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA,
                    GPIO_PIN_4 |
                    GPIO_PIN_5 |
                    GPIO_PIN_6 |
                    GPIO_PIN_7);
}

/**
 * @brief 按指定 SPI 模式、相位和分频配置并启动 SPI1。
 * @param polarity 时钟极性。
 * @param phase 时钟相位。
 * @param prescaler SPI 波特率预分频值。
 * @retval GYRO_SPI_OK 配置成功。
 * @retval GYRO_SPI_ERROR HAL_SPI_Init 失败。
 */
uint8_t gyro_spi_configure(uint32_t polarity,
                           uint32_t phase,
                           uint32_t prescaler)
{
    hgyro_spi.Instance = SPI1;
    hgyro_spi.Init.Mode = SPI_MODE_MASTER;
    hgyro_spi.Init.Direction = SPI_DIRECTION_2LINES;
    hgyro_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    hgyro_spi.Init.CLKPolarity = polarity;
    hgyro_spi.Init.CLKPhase = phase;
    hgyro_spi.Init.NSS = SPI_NSS_SOFT;
    hgyro_spi.Init.BaudRatePrescaler = prescaler;
    hgyro_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hgyro_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    hgyro_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hgyro_spi.Init.CRCPolynomial = 10U;
    hgyro_spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hgyro_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hgyro_spi) != HAL_OK)
    {
        return GYRO_SPI_ERROR;
    }

    gyro_spi_cs_high();
    return GYRO_SPI_OK;
}

/**
 * @brief 以 LSM6 系列传感器的 SPI Mode 3 默认参数初始化 SPI1。
 * @return GYRO_SPI_OK 表示成功，否则返回 gyro_spi_configure 的错误码。
 */
uint8_t gyro_spi_init(void)
{
    /* LSM6-family sensors use SPI mode 3 for normal 4-wire access. */
    return gyro_spi_configure(SPI_POLARITY_HIGH,
                              SPI_PHASE_2EDGE,
                              SPI_BAUDRATEPRESCALER_32);
}

/**
 * @brief 拉低 PA4 片选，使陀螺仪开始一次 SPI 事务。
 */
void gyro_spi_cs_low(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 拉高 PA4 片选，结束当前陀螺仪 SPI 事务。
 */
void gyro_spi_cs_high(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

/**
 * @brief 在 SPI1 上执行一次阻塞式全双工收发，并自动控制片选。
 * @param tx_data 发送缓冲区。
 * @param rx_data 接收缓冲区。
 * @param length 传输字节数。
 * @retval GYRO_SPI_OK 传输成功。
 * @retval GYRO_SPI_BAD_PARAM 参数为空或长度为零。
 * @retval GYRO_SPI_ERROR HAL 传输失败或超时。
 */
uint8_t gyro_spi_transfer(const uint8_t *tx_data,
                          uint8_t *rx_data,
                          uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((tx_data == 0) || (rx_data == 0) || (length == 0U))
    {
        return GYRO_SPI_BAD_PARAM;
    }

    gyro_spi_cs_low();
    status = HAL_SPI_TransmitReceive(&hgyro_spi,
                                     (uint8_t *)tx_data,
                                     rx_data,
                                     length,
                                     GYRO_SPI_TIMEOUT_MS);
    gyro_spi_cs_high();

    return (status == HAL_OK) ? GYRO_SPI_OK : GYRO_SPI_ERROR;
}
