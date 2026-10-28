#include "gyro_dma.h"
#include "gyro_spi.h"

#define GYRO_DMA_RX_STREAM       DMA2_Stream0
#define GYRO_DMA_TX_STREAM       DMA2_Stream3
#define GYRO_DMA_CHANNEL         DMA_CHANNEL_3
#define GYRO_DMA_IRQ_PRIORITY    6U
#define GYRO_DMA_ALIGNMENT_MASK  0x1FU

DMA_HandleTypeDef hgyro_dma_rx;
DMA_HandleTypeDef hgyro_dma_tx;

static SPI_HandleTypeDef *gyro_spi;
static const uint8_t *active_tx_data;
static uint8_t *active_rx_data;
static uint16_t active_length;
static volatile uint8_t dma_complete;
static volatile uint8_t dma_error;
static volatile uint8_t dma_busy;
static uint8_t dma_initialized;

/**
 * @brief 初始化一个 SPI 字节搬运用的 DMA 数据流。
 * @param dma HAL DMA 句柄。
 * @param stream 要绑定的 DMA 数据流。
 * @param direction 数据流方向，外设到内存或内存到外设。
 * @return GYRO_DMA_OK 表示初始化成功，否则返回 GYRO_DMA_ERROR。
 */
static uint8_t init_dma_stream(DMA_HandleTypeDef *dma,
                               DMA_Stream_TypeDef *stream,
                               uint32_t direction)
{
    dma->Instance = stream;
    dma->Init.Channel = GYRO_DMA_CHANNEL;
    dma->Init.Direction = direction;
    dma->Init.PeriphInc = DMA_PINC_DISABLE;
    dma->Init.MemInc = DMA_MINC_ENABLE;
    dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma->Init.Mode = DMA_NORMAL;
    dma->Init.Priority = DMA_PRIORITY_HIGH;
    dma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    dma->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    dma->Init.MemBurst = DMA_MBURST_SINGLE;
    dma->Init.PeriphBurst = DMA_PBURST_SINGLE;

    return (HAL_DMA_Init(dma) == HAL_OK) ? GYRO_DMA_OK : GYRO_DMA_ERROR;
}

/**
 * @brief 初始化 SPI1 使用的 DMA2 Stream0/3、关联句柄并开启中断。
 * @param spi 已配置的 SPI1 句柄。
 * @retval GYRO_DMA_OK 初始化成功或已经初始化。
 * @retval GYRO_DMA_BAD_PARAM 句柄为空或不是 SPI1。
 * @retval GYRO_DMA_ERROR DMA 数据流初始化失败。
 */
uint8_t gyro_dma_init(SPI_HandleTypeDef *spi)
{
    if ((spi == 0) || (spi->Instance != SPI1))
    {
        return GYRO_DMA_BAD_PARAM;
    }

    if (dma_initialized != 0U)
    {
        gyro_spi = spi;
        return GYRO_DMA_OK;
    }

    __HAL_RCC_DMA2_CLK_ENABLE();

    if ((init_dma_stream(&hgyro_dma_rx,
                         GYRO_DMA_RX_STREAM,
                         DMA_PERIPH_TO_MEMORY) != GYRO_DMA_OK) ||
        (init_dma_stream(&hgyro_dma_tx,
                         GYRO_DMA_TX_STREAM,
                         DMA_MEMORY_TO_PERIPH) != GYRO_DMA_OK))
    {
        return GYRO_DMA_ERROR;
    }

    __HAL_LINKDMA(spi, hdmarx, hgyro_dma_rx);
    __HAL_LINKDMA(spi, hdmatx, hgyro_dma_tx);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn,
                         GYRO_DMA_IRQ_PRIORITY,
                         0U);
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn,
                         GYRO_DMA_IRQ_PRIORITY,
                         0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

    gyro_spi = spi;
    dma_complete = 0U;
    dma_error = 0U;
    dma_busy = 0U;
    dma_initialized = 1U;
    return GYRO_DMA_OK;
}

/**
 * @brief 停止并释放两个陀螺仪 DMA 数据流及其 NVIC 中断。
 * @param spi 与 DMA 关联的 SPI1 句柄。
 */
void gyro_dma_deinit(SPI_HandleTypeDef *spi)
{
    if ((spi == 0) || (spi->Instance != SPI1))
    {
        return;
    }

    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    (void)HAL_DMA_DeInit(&hgyro_dma_rx);
    (void)HAL_DMA_DeInit(&hgyro_dma_tx);
    spi->hdmarx = 0;
    spi->hdmatx = 0;
    gyro_spi = 0;
    dma_complete = 0U;
    dma_error = 0U;
    dma_busy = 0U;
    dma_initialized = 0U;
}

/**
 * @brief 启动一次由 SPI1 驱动的 DMA 全双工传输。
 * @param tx_data 32 字节对齐的发送缓冲区。
 * @param rx_data 32 字节对齐的接收缓冲区。
 * @param length 传输字节数。
 * @retval GYRO_DMA_OK 传输已启动。
 * @retval GYRO_DMA_BAD_PARAM 参数无效或缓冲区未对齐。
 * @retval GYRO_DMA_BUSY 已有 DMA 传输正在进行。
 * @retval GYRO_DMA_ERROR HAL 启动传输失败。
 * @note 启动前会处理 Cortex-M7 数据缓存的一致性。
 */
uint8_t gyro_dma_start(const uint8_t *tx_data,
                       uint8_t *rx_data,
                       uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((tx_data == 0) ||
        (rx_data == 0) ||
        (length == 0U))
    {
        return GYRO_DMA_BAD_PARAM;
    }

    if ((((uint32_t)tx_data) & GYRO_DMA_ALIGNMENT_MASK) != 0U ||
        (((uint32_t)rx_data) & GYRO_DMA_ALIGNMENT_MASK) != 0U)
    {
        return GYRO_DMA_BAD_PARAM;
    }

    if (dma_busy != 0U)
    {
        return GYRO_DMA_BUSY;
    }

    active_tx_data = tx_data;
    active_rx_data = rx_data;
    active_length = length;
    dma_complete = 0U;
    dma_error = 0U;
    dma_busy = 1U;

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_CleanDCache_by_Addr((uint32_t *)active_tx_data, active_length);
    SCB_InvalidateDCache_by_Addr((uint32_t *)active_rx_data, active_length);
#endif

    gyro_spi_cs_low();
    status = HAL_SPI_TransmitReceive_DMA(gyro_spi,
                                         (uint8_t *)active_tx_data,
                                         active_rx_data,
                                         active_length);
    if (status != HAL_OK)
    {
        gyro_spi_cs_high();
        dma_busy = 0U;
        dma_error = 1U;
        return GYRO_DMA_ERROR;
    }

    return GYRO_DMA_OK;
}

/**
 * @brief 原子读取并清除 DMA 完成标志。
 * @return 1 表示上次轮询后发生过完成事件，0 表示没有。
 */
uint8_t gyro_dma_take_complete(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t complete;

    __disable_irq();
    complete = dma_complete;
    dma_complete = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return complete;
}

/**
 * @brief 原子读取并清除 DMA 错误标志。
 * @return 1 表示发生过 DMA/SPI 错误，0 表示没有。
 */
uint8_t gyro_dma_take_error(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t error;

    __disable_irq();
    error = dma_error;
    dma_error = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return error;
}

/**
 * @brief 查询当前是否仍有 DMA 传输占用 SPI1。
 * @return 1 表示忙，0 表示空闲。
 */
uint8_t gyro_dma_is_busy(void)
{
    return dma_busy;
}

/**
 * @brief 中止正在进行的 DMA 传输、释放片选并标记错误状态。
 * @retval GYRO_DMA_OK 中止成功。
 * @retval GYRO_DMA_ERROR 当前没有可中止的传输或 SPI 未初始化。
 */
uint8_t gyro_dma_abort(void)
{
    if ((gyro_spi == 0) || (dma_busy == 0U))
    {
        return GYRO_DMA_ERROR;
    }

    (void)HAL_SPI_Abort(gyro_spi);
    gyro_spi_cs_high();
    dma_busy = 0U;
    dma_error = 1U;
    return GYRO_DMA_OK;
}

/**
 * @brief SPI 全双工 DMA 完成回调，释放片选并通知上层读取结果。
 * @param spi 产生回调的 SPI 句柄。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if ((spi != gyro_spi) || (dma_busy == 0U))
    {
        return;
    }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)active_rx_data, active_length);
#endif

    gyro_spi_cs_high();
    dma_busy = 0U;
    dma_complete = 1U;
}

/**
 * @brief SPI 或 DMA 错误回调，释放片选并记录传输错误。
 * @param spi 产生错误回调的 SPI 句柄。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if ((spi != gyro_spi) || (dma_busy == 0U))
    {
        return;
    }

    gyro_spi_cs_high();
    dma_busy = 0U;
    dma_error = 1U;
}

/**
 * @brief DMA2 Stream0 中断入口，用于处理 SPI1 接收 DMA 事件。
 */
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hgyro_dma_rx);
}

/**
 * @brief DMA2 Stream3 中断入口，用于处理 SPI1 发送 DMA 事件。
 */
void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hgyro_dma_tx);
}
