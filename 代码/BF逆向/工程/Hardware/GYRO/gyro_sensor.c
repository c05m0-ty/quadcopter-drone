#include "gyro_sensor.h"
#include "gyro_dma.h"
#include "gyro_spi.h"

#define GYRO_REG_WHO_AM_I       0x0FU
#define GYRO_SPI_READ_BIT       0x80U
#define GYRO_STARTUP_DELAY_MS   5U

#define GYRO_REG_INT1_CTRL      0x0DU
#define GYRO_REG_CTRL1_XL       0x10U
#define GYRO_REG_CTRL2_G        0x11U
#define GYRO_REG_CTRL3_C        0x12U
#define GYRO_REG_CTRL4_C        0x13U
#define GYRO_REG_CTRL6_G        0x15U
#define GYRO_REG_CTRL7_G        0x16U
#define GYRO_REG_CTRL8_XL       0x17U
#define GYRO_REG_CTRL9_XL       0x18U
#define GYRO_REG_FIFO_CTRL1     0x07U
#define GYRO_REG_FIFO_CTRL2     0x08U
#define GYRO_REG_FIFO_CTRL3     0x09U
#define GYRO_REG_FIFO_CTRL4     0x0AU
#define GYRO_REG_FIFO_STATUS1   0x1BU
#define GYRO_REG_FIFO_STATUS2   0x1CU
#define GYRO_REG_FIFO_DATA_TAG  0x78U
#define GYRO_REG_HAODR_CFG      0x62U
#define GYRO_REG_OUTX_L_G       0x22U

#define LSM6DSK320X_ID          0x75U
#define CTRL3_SW_RESET          0x01U
#define CTRL3_IF_INC_AND_BDU    0x44U
#define HAODR_MODE1             0x01U
#define CTRL8_FS_XL_16G         0x03U
#define CTRL6_FS_G_2000DPS_BW288 0x04U
#define CTRL1_XL_1000HZ         0x19U
#define CTRL2_G_1000HZ          0x19U
#define CTRL7_LPF1_G_EN         0x01U
#define CTRL9_LPF2_XL_EN        0x08U
#define CTRL4_DRDY_PULSED       0x02U
#define INT1_DRDY_G             0x02U
#define INT1_FIFO_TH            0x08U
#define INT1_FIFO_OVR           0x10U
#define FIFO_CTRL3_RATE_960HZ   0x09U
#define FIFO_MODE_BYPASS        0x00U
#define FIFO_MODE_CONTINUOUS    0x06U

#define FIFO_TAG_GYRO           1U
#define FIFO_TAG_ACCEL          2U
#define FIFO_TAG_TEMP           3U
#define FIFO_TAG_TIMESTAMP      4U
#define FIFO_TAG_CONFIG_CHANGE  5U

static volatile uint8_t gyro_data_ready;

#if defined(__CC_ARM)
#define GYRO_DMA_ALIGN __align(32)
#elif defined(__GNUC__)
#define GYRO_DMA_ALIGN __attribute__((aligned(32)))
#else
#define GYRO_DMA_ALIGN
#endif

static GYRO_DMA_ALIGN uint8_t gyro_dma_tx_data[13];
static GYRO_DMA_ALIGN uint8_t gyro_dma_rx_data[13];
static GYRO_DMA_ALIGN uint8_t
    gyro_fifo_tx_data[(GYRO_FIFO_MAX_RECORDS * 7U) + 1U];
static GYRO_DMA_ALIGN uint8_t
    gyro_fifo_rx_data[(GYRO_FIFO_MAX_RECORDS * 7U) + 1U];
static uint16_t gyro_fifo_active_records;

static void gyro_sensor_parse_sample(const uint8_t *data,
                                     gyro_sensor_sample_t *sample)
{
    sample->gyro_x = (int16_t)((uint16_t)data[0] |
                              ((uint16_t)data[1] << 8));
    sample->gyro_y = (int16_t)((uint16_t)data[2] |
                              ((uint16_t)data[3] << 8));
    sample->gyro_z = (int16_t)((uint16_t)data[4] |
                              ((uint16_t)data[5] << 8));
    sample->accel_x = (int16_t)((uint16_t)data[6] |
                               ((uint16_t)data[7] << 8));
    sample->accel_y = (int16_t)((uint16_t)data[8] |
                               ((uint16_t)data[9] << 8));
    sample->accel_z = (int16_t)((uint16_t)data[10] |
                               ((uint16_t)data[11] << 8));
}

static void gyro_sensor_irq_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);

    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 6U, 0U);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

static uint8_t gyro_sensor_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = reg;
    tx_data[1] = value;

    return gyro_spi_transfer(tx_data, rx_data, sizeof(tx_data));
}

static uint8_t gyro_sensor_read_registers(uint8_t reg,
                                          uint8_t *data,
                                          uint16_t length)
{
    uint8_t tx_data[13];
    uint8_t rx_data[13];
    uint16_t index;

    if ((data == 0) || (length == 0U) || (length > 12U))
    {
        return GYRO_SPI_BAD_PARAM;
    }

    tx_data[0] = (uint8_t)(reg | GYRO_SPI_READ_BIT);
    for (index = 1U; index <= length; index++)
    {
        tx_data[index] = 0xFFU;
    }

    if (gyro_spi_transfer(tx_data, rx_data, (uint16_t)(length + 1U)) !=
        GYRO_SPI_OK)
    {
        return GYRO_SPI_ERROR;
    }

    for (index = 0U; index < length; index++)
    {
        data[index] = rx_data[index + 1U];
    }

    return GYRO_SPI_OK;
}

static uint8_t gyro_sensor_read_identity(uint32_t polarity,
                                         uint32_t phase,
                                         uint32_t prescaler,
                                         uint8_t *identity)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    if (identity == 0)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    *identity = 0U;

    if (gyro_spi_configure(polarity, phase, prescaler) != GYRO_SPI_OK)
    {
        return GYRO_SENSOR_SPI_INIT_FAIL;
    }

    HAL_Delay(GYRO_STARTUP_DELAY_MS);

    tx_data[0] = (uint8_t)(GYRO_REG_WHO_AM_I | GYRO_SPI_READ_BIT);
    tx_data[1] = 0xFFU;

    if (gyro_spi_transfer(tx_data, rx_data, sizeof(tx_data)) != GYRO_SPI_OK)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    *identity = rx_data[1];
    return GYRO_SENSOR_OK;
}

static uint8_t gyro_sensor_read_identity_retry(uint32_t polarity,
                                               uint32_t phase,
                                               uint32_t prescaler,
                                               uint8_t *identity)
{
    uint32_t retry;

    for (retry = 0U; retry < 5U; retry++)
    {
        if (gyro_sensor_read_identity(polarity,
                                      phase,
                                      prescaler,
                                      identity) != GYRO_SENSOR_OK)
        {
            return GYRO_SENSOR_READ_FAIL;
        }

        if ((*identity != 0x00U) && (*identity != 0xFFU))
        {
            return GYRO_SENSOR_OK;
        }

        HAL_Delay(2U);
    }

    return GYRO_SENSOR_OK;
}

uint8_t gyro_sensor_probe(uint8_t identities[GYRO_PROBE_COUNT])
{
    uint8_t status = GYRO_SENSOR_OK;

    if (identities == 0)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    identities[GYRO_PROBE_MODE3_FAST] = 0U;
    identities[GYRO_PROBE_MODE3_SLOW] = 0U;
    identities[GYRO_PROBE_MODE0_FAST] = 0U;
    identities[GYRO_PROBE_MODE0_SLOW] = 0U;

    if (gyro_sensor_read_identity(SPI_POLARITY_HIGH,
                                  SPI_PHASE_2EDGE,
                                  SPI_BAUDRATEPRESCALER_32,
                                  &identities[GYRO_PROBE_MODE3_FAST]) !=
        GYRO_SENSOR_OK)
    {
        status = GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_sensor_read_identity(SPI_POLARITY_HIGH,
                                  SPI_PHASE_2EDGE,
                                  SPI_BAUDRATEPRESCALER_128,
                                  &identities[GYRO_PROBE_MODE3_SLOW]) !=
        GYRO_SENSOR_OK)
    {
        status = GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_sensor_read_identity(SPI_POLARITY_LOW,
                                  SPI_PHASE_1EDGE,
                                  SPI_BAUDRATEPRESCALER_32,
                                  &identities[GYRO_PROBE_MODE0_FAST]) !=
        GYRO_SENSOR_OK)
    {
        status = GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_sensor_read_identity(SPI_POLARITY_LOW,
                                  SPI_PHASE_1EDGE,
                                  SPI_BAUDRATEPRESCALER_128,
                                  &identities[GYRO_PROBE_MODE0_SLOW]) !=
        GYRO_SENSOR_OK)
    {
        status = GYRO_SENSOR_READ_FAIL;
    }

    return status;
}

uint8_t gyro_sensor_init(uint8_t *identity)
{
    uint8_t status;

    if (identity == 0)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    status = gyro_sensor_read_identity_retry(SPI_POLARITY_HIGH,
                                             SPI_PHASE_2EDGE,
                                             SPI_BAUDRATEPRESCALER_128,
                                             identity);
    if (status != GYRO_SENSOR_OK)
    {
        return status;
    }

    if ((*identity != LSM6DSK320X_ID) &&
        (gyro_sensor_read_identity_retry(SPI_POLARITY_LOW,
                                         SPI_PHASE_1EDGE,
                                         SPI_BAUDRATEPRESCALER_128,
                                         identity) != GYRO_SENSOR_OK))
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    if (*identity != LSM6DSK320X_ID)
    {
        return GYRO_SENSOR_WRONG_ID;
    }

    if (gyro_sensor_write_register(GYRO_REG_CTRL3_C,
                                   CTRL3_SW_RESET) != GYRO_SPI_OK)
    {
        return GYRO_SENSOR_RESET_FAIL;
    }

    HAL_Delay(10U);

    if ((gyro_sensor_write_register(GYRO_REG_CTRL3_C,
                                    CTRL3_IF_INC_AND_BDU) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_HAODR_CFG,
                                    HAODR_MODE1) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL8_XL,
                                    CTRL8_FS_XL_16G) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL6_G,
                                    CTRL6_FS_G_2000DPS_BW288) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL7_G,
                                    CTRL7_LPF1_G_EN) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL9_XL,
                                    CTRL9_LPF2_XL_EN) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL4_C,
                                    CTRL4_DRDY_PULSED) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_INT1_CTRL,
                                    0U) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL1_XL,
                                    CTRL1_XL_1000HZ) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_CTRL2_G,
                                    CTRL2_G_1000HZ) != GYRO_SPI_OK))
    {
        return GYRO_SENSOR_CONFIG_FAIL;
    }

    if (gyro_sensor_write_register(GYRO_REG_FIFO_CTRL4,
                                   FIFO_MODE_BYPASS) != GYRO_SPI_OK)
    {
        return GYRO_SENSOR_CONFIG_FAIL;
    }

    gyro_data_ready = 0U;
    gyro_sensor_irq_init();

    HAL_Delay(10U);
    return GYRO_SENSOR_OK;
}

uint8_t gyro_sensor_enable_fifo(void)
{
    gyro_data_ready = 0U;

    if ((gyro_sensor_write_register(GYRO_REG_FIFO_CTRL4,
                                    FIFO_MODE_BYPASS) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_FIFO_CTRL1,
                                    GYRO_FIFO_WATERMARK) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_FIFO_CTRL2,
                                    0U) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(
            GYRO_REG_FIFO_CTRL3,
            (uint8_t)((FIFO_CTRL3_RATE_960HZ << 4) |
                      FIFO_CTRL3_RATE_960HZ)) != GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_INT1_CTRL,
                                    INT1_FIFO_TH | INT1_FIFO_OVR) !=
                                    GYRO_SPI_OK) ||
        (gyro_sensor_write_register(GYRO_REG_FIFO_CTRL4,
                                    FIFO_MODE_CONTINUOUS) != GYRO_SPI_OK))
    {
        return GYRO_SENSOR_CONFIG_FAIL;
    }

    return GYRO_SENSOR_OK;
}

uint8_t gyro_sensor_read(gyro_sensor_sample_t *sample)
{
    uint8_t data[12];

    if (sample == 0)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_sensor_read_registers(GYRO_REG_OUTX_L_G,
                                   data,
                                   sizeof(data)) != GYRO_SPI_OK)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    gyro_sensor_parse_sample(data, sample);

    return GYRO_SENSOR_OK;
}

uint8_t gyro_sensor_start_dma_read(void)
{
    uint16_t index;

    gyro_dma_tx_data[0] =
        (uint8_t)(GYRO_REG_OUTX_L_G | GYRO_SPI_READ_BIT);
    for (index = 1U; index < sizeof(gyro_dma_tx_data); index++)
    {
        gyro_dma_tx_data[index] = 0xFFU;
    }

    return (gyro_dma_start(gyro_dma_tx_data,
                           gyro_dma_rx_data,
                           sizeof(gyro_dma_tx_data)) == GYRO_DMA_OK)
        ? GYRO_SENSOR_OK
        : ((gyro_dma_is_busy() != 0U)
            ? GYRO_SENSOR_DMA_BUSY
            : GYRO_SENSOR_READ_FAIL);
}

uint8_t gyro_sensor_finish_dma_read(gyro_sensor_sample_t *sample)
{
    if (sample == 0)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_dma_take_error() != 0U)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_dma_take_complete() == 0U)
    {
        return GYRO_SENSOR_DMA_BUSY;
    }

    gyro_sensor_parse_sample(&gyro_dma_rx_data[1], sample);
    return GYRO_SENSOR_OK;
}

static uint8_t gyro_sensor_fifo_payload_length(uint8_t tag_sensor)
{
    switch (tag_sensor)
    {
    case FIFO_TAG_GYRO:
    case FIFO_TAG_ACCEL:
        return 6U;

    case FIFO_TAG_TEMP:
        return 2U;

    case FIFO_TAG_TIMESTAMP:
        return 4U;

    case FIFO_TAG_CONFIG_CHANGE:
        return 0U;

    default:
        return 0xFFU;
    }
}

uint8_t gyro_sensor_start_fifo_dma_read(void)
{
    uint8_t fifo_status[2];
    uint32_t active_length;
    uint32_t index;
    uint16_t record_count;

    if (gyro_dma_is_busy() != 0U)
    {
        return GYRO_SENSOR_DMA_BUSY;
    }

    if (gyro_sensor_read_registers(GYRO_REG_FIFO_STATUS1,
                                   fifo_status,
                                   sizeof(fifo_status)) != GYRO_SPI_OK)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    record_count = (uint16_t)fifo_status[0] |
                   (uint16_t)((fifo_status[1] & 0x01U) << 8);
    if (record_count == 0U)
    {
        return GYRO_SENSOR_NO_DATA;
    }

    if (record_count > GYRO_FIFO_MAX_RECORDS)
    {
        record_count = GYRO_FIFO_MAX_RECORDS;
    }

    gyro_fifo_active_records = record_count;
    active_length = ((uint32_t)record_count * 7U) + 1U;
    gyro_fifo_tx_data[0] =
        (uint8_t)(GYRO_REG_FIFO_DATA_TAG | GYRO_SPI_READ_BIT);
    for (index = 1U; index < active_length; index++)
    {
        gyro_fifo_tx_data[index] = 0xFFU;
    }

    return (gyro_dma_start(gyro_fifo_tx_data,
                           gyro_fifo_rx_data,
                           (uint16_t)active_length) == GYRO_DMA_OK)
        ? GYRO_SENSOR_OK
        : GYRO_SENSOR_READ_FAIL;
}

uint8_t gyro_sensor_finish_fifo_dma_read(gyro_sensor_fifo_batch_t *batch)
{
    uint16_t record;
    uint16_t offset = 1U;
    uint16_t active_length =
        (uint16_t)((gyro_fifo_active_records * 7U) + 1U);

    if (batch == 0)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_dma_take_error() != 0U)
    {
        return GYRO_SENSOR_READ_FAIL;
    }

    if (gyro_dma_take_complete() == 0U)
    {
        return GYRO_SENSOR_DMA_BUSY;
    }

    batch->gyro_count = 0U;
    batch->accel_valid = 0U;

    for (record = 0U;
         record < gyro_fifo_active_records;
         record++)
    {
        uint8_t tag;
        uint8_t tag_sensor;
        uint8_t payload_length;

        if (offset >= active_length)
        {
            break;
        }

        tag = gyro_fifo_rx_data[offset++];
        tag_sensor = (uint8_t)((tag & 0xF8U) >> 3);
        if (tag_sensor == 0U)
        {
            break;
        }

        payload_length =
            gyro_sensor_fifo_payload_length(tag_sensor);
        if ((payload_length == 0xFFU) ||
            ((uint32_t)offset + payload_length > active_length))
        {
            return GYRO_SENSOR_READ_FAIL;
        }

        if ((tag_sensor == FIFO_TAG_GYRO) &&
            (batch->gyro_count < GYRO_FIFO_MAX_RECORDS))
        {
            gyro_sensor_sample_t *sample =
                &batch->gyro_samples[batch->gyro_count++];

            sample->gyro_x = (int16_t)((uint16_t)
                gyro_fifo_rx_data[offset] |
                ((uint16_t)gyro_fifo_rx_data[offset + 1U] << 8));
            sample->gyro_y = (int16_t)((uint16_t)
                gyro_fifo_rx_data[offset + 2U] |
                ((uint16_t)gyro_fifo_rx_data[offset + 3U] << 8));
            sample->gyro_z = (int16_t)((uint16_t)
                gyro_fifo_rx_data[offset + 4U] |
                ((uint16_t)gyro_fifo_rx_data[offset + 5U] << 8));
            sample->accel_x = 0;
            sample->accel_y = 0;
            sample->accel_z = 0;
        }
        else if (tag_sensor == FIFO_TAG_ACCEL)
        {
            batch->accel_sample.accel_x = (int16_t)((uint16_t)
                gyro_fifo_rx_data[offset] |
                ((uint16_t)gyro_fifo_rx_data[offset + 1U] << 8));
            batch->accel_sample.accel_y = (int16_t)((uint16_t)
                gyro_fifo_rx_data[offset + 2U] |
                ((uint16_t)gyro_fifo_rx_data[offset + 3U] << 8));
            batch->accel_sample.accel_z = (int16_t)((uint16_t)
                gyro_fifo_rx_data[offset + 4U] |
                ((uint16_t)gyro_fifo_rx_data[offset + 5U] << 8));
            batch->accel_valid = 1U;
        }

        offset = (uint16_t)(offset + payload_length);
    }

    return GYRO_SENSOR_OK;
}

void gyro_sensor_abort_dma_read(void)
{
    (void)gyro_dma_abort();
}

uint8_t gyro_sensor_take_data_ready(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t ready;

    __disable_irq();
    ready = gyro_data_ready;
    gyro_data_ready = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return ready;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3)
    {
        gyro_data_ready = 1U;
    }
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

int32_t gyro_sensor_raw_to_centidps(int16_t raw)
{
    return (int32_t)raw * 7;
}

int32_t gyro_sensor_raw_to_centig(int16_t raw)
{
    return ((int32_t)raw * 100) / 2048;
}
