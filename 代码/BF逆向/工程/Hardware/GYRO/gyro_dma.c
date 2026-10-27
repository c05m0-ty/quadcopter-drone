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

uint8_t gyro_dma_is_busy(void)
{
    return dma_busy;
}

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

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hgyro_dma_rx);
}

void DMA2_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hgyro_dma_tx);
}
