#include "power_adc.h"

#define POWER_ADC_CURRENT_CHANNEL   ADC_CHANNEL_11
#define POWER_ADC_VOLTAGE_CHANNEL   ADC_CHANNEL_12
#define POWER_ADC_DMA_STREAM        DMA2_Stream1
#define POWER_ADC_DMA_CHANNEL       DMA_CHANNEL_2
#define POWER_ADC_DMA_IRQ_PRIORITY  7U
#define POWER_ADC_VREF_MV           3300L
#define POWER_ADC_VBAT_SCALE        110L
#define POWER_ADC_VBAT_DIVIDER      10L
#define POWER_ADC_IBATA_SCALE       264L
#define POWER_ADC_IBATA_OFFSET      424L
#define POWER_ADC_MIN_VALID_VOLTAGE 1.0f
#define POWER_ADC_VOLTAGE_FILTER    0.20f
#define POWER_ADC_CURRENT_FILTER    0.10f

#if defined(__CC_ARM)
#define POWER_ADC_DMA_ALIGN __align(32)
#elif defined(__GNUC__)
#define POWER_ADC_DMA_ALIGN __attribute__((aligned(32)))
#else
#define POWER_ADC_DMA_ALIGN
#endif

ADC_HandleTypeDef hpower_adc;
DMA_HandleTypeDef hpower_adc_dma;

static POWER_ADC_DMA_ALIGN uint16_t power_adc_dma_buffer[2];
static uint8_t power_adc_dma_ready;
static float power_adc_voltage_filtered;
static float power_adc_current_filtered;
static uint8_t power_adc_filter_initialized;

/**
 * @brief 初始化 ADC3、PC1/PC2 模拟输入及其 DMA 底层资源。
 * @param hadc HAL ADC 句柄；仅处理 ADC3。
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc->Instance != ADC3)
    {
        return;
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    hpower_adc_dma.Instance = POWER_ADC_DMA_STREAM;
    hpower_adc_dma.Init.Channel = POWER_ADC_DMA_CHANNEL;
    hpower_adc_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hpower_adc_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    hpower_adc_dma.Init.MemInc = DMA_MINC_ENABLE;
    hpower_adc_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hpower_adc_dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hpower_adc_dma.Init.Mode = DMA_CIRCULAR;
    hpower_adc_dma.Init.Priority = DMA_PRIORITY_MEDIUM;
    hpower_adc_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hpower_adc_dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hpower_adc_dma.Init.MemBurst = DMA_MBURST_SINGLE;
    hpower_adc_dma.Init.PeriphBurst = DMA_PBURST_SINGLE;

    power_adc_dma_ready =
        (HAL_DMA_Init(&hpower_adc_dma) == HAL_OK) ? 1U : 0U;

    __HAL_LINKDMA(hadc, DMA_Handle, hpower_adc_dma);

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn,
                         POWER_ADC_DMA_IRQ_PRIORITY,
                         0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/**
 * @brief 初始化 ADC3 双通道连续扫描，并启动循环 DMA。
 * @retval POWER_ADC_OK 初始化和 DMA 启动成功。
 * @retval POWER_ADC_DMA_INIT_FAIL DMA 底层初始化失败。
 * @retval POWER_ADC_INIT_FAIL ADC3 初始化失败。
 * @retval POWER_ADC_CHANNEL_FAIL ADC 通道配置失败。
 * @retval POWER_ADC_START_FAIL ADC DMA 启动失败。
 * @note 扫描顺序固定为电流通道 11、电压通道 12。
 */
uint8_t power_adc_init(void)
{
    ADC_ChannelConfTypeDef channel = {0};

    hpower_adc.Instance = ADC3;
    hpower_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hpower_adc.Init.Resolution = ADC_RESOLUTION_12B;
    hpower_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hpower_adc.Init.ScanConvMode = ENABLE;
    hpower_adc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hpower_adc.Init.ContinuousConvMode = ENABLE;
    hpower_adc.Init.NbrOfConversion = 2U;
    hpower_adc.Init.DiscontinuousConvMode = DISABLE;
    hpower_adc.Init.NbrOfDiscConversion = 1U;
    hpower_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hpower_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hpower_adc.Init.DMAContinuousRequests = ENABLE;

    if (HAL_ADC_Init(&hpower_adc) != HAL_OK)
    {
        return POWER_ADC_INIT_FAIL;
    }

    if (power_adc_dma_ready == 0U)
    {
        return POWER_ADC_DMA_INIT_FAIL;
    }

    channel.Channel = POWER_ADC_CURRENT_CHANNEL;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    channel.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hpower_adc, &channel) != HAL_OK)
    {
        return POWER_ADC_CHANNEL_FAIL;
    }

    channel.Channel = POWER_ADC_VOLTAGE_CHANNEL;
    channel.Rank = 2U;
    if (HAL_ADC_ConfigChannel(&hpower_adc, &channel) != HAL_OK)
    {
        return POWER_ADC_CHANNEL_FAIL;
    }

    if (HAL_ADC_Start_DMA(&hpower_adc,
                          (uint32_t *)power_adc_dma_buffer,
                          2U) != HAL_OK)
    {
        return POWER_ADC_START_FAIL;
    }

    return POWER_ADC_OK;
}

/**
 * @brief 复制最近一次 ADC 原始采样，DMA 会在后台持续刷新。
 * @param sample 电流和电压原始值输出；为空时直接返回。
 */
void power_adc_get_raw(power_adc_raw_t *sample)
{
    uint32_t primask;

    if (sample == 0)
    {
        return;
    }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((uint32_t *)power_adc_dma_buffer,
                                 sizeof(power_adc_dma_buffer));
#endif

    primask = __get_PRIMASK();
    __disable_irq();
    sample->current_raw = power_adc_dma_buffer[0];
    sample->voltage_raw = power_adc_dma_buffer[1];
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief 将 ADC 原始值换算为电池电压和电流，并进行一阶低通滤波。
 * @param sample 校准后的电压、电流及对应原始值输出。
 * @note 电压换算参数来自该板 Betaflight 配置：
 *       scale=110、divider=10、Vref=3.3V。
 *       电流参数为 scale=264、offset=424。
 *       电池电压低于 1V 时认为主电未接入，电流强制为 0A。
 */
void power_adc_get_measurement(power_measurement_t *sample)
{
    power_adc_raw_t raw;
    uint32_t voltage_numerator;
    uint32_t voltage_centivolts;
    int32_t current_millivolts;
    int32_t current_centiamps;
    float voltage;
    float current;

    if (sample == 0)
    {
        return;
    }

    power_adc_get_raw(&raw);

    voltage_numerator =
        ((uint32_t)raw.voltage_raw *
         (uint32_t)POWER_ADC_VBAT_SCALE *
         (uint32_t)POWER_ADC_VREF_MV / 10U) +
        (4095U * 5U);
    voltage_centivolts =
        voltage_numerator /
        (4095U * (uint32_t)POWER_ADC_VBAT_DIVIDER);
    voltage = (float)voltage_centivolts * 0.01f;

    current_millivolts =
        ((int32_t)raw.current_raw * POWER_ADC_VREF_MV) / 4096L;
    current_centiamps =
        ((current_millivolts * 10000L) / POWER_ADC_IBATA_SCALE +
         POWER_ADC_IBATA_OFFSET) / 10L;
    current = (float)current_centiamps * 0.01f;

    if ((voltage < POWER_ADC_MIN_VALID_VOLTAGE) || (current < 0.0f))
    {
        current = 0.0f;
    }

    if (power_adc_filter_initialized == 0U)
    {
        power_adc_voltage_filtered = voltage;
        power_adc_current_filtered = current;
        power_adc_filter_initialized = 1U;
    }
    else
    {
        power_adc_voltage_filtered +=
            POWER_ADC_VOLTAGE_FILTER *
            (voltage - power_adc_voltage_filtered);
        power_adc_current_filtered +=
            POWER_ADC_CURRENT_FILTER *
            (current - power_adc_current_filtered);
    }

    sample->current_raw = raw.current_raw;
    sample->voltage_raw = raw.voltage_raw;
    sample->voltage_v = power_adc_voltage_filtered;
    sample->current_a = power_adc_current_filtered;
}

/**
 * @brief DMA2 Stream1 中断入口，用于处理 ADC3 接收 DMA 事件。
 */
void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hpower_adc_dma);
}
