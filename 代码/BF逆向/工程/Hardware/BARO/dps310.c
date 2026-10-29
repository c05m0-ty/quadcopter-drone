#include "dps310.h"
#include <math.h>

#define DPS310_I2C_ADDRESS          0x76U
#define DPS310_I2C_ADDRESS_8BIT     (DPS310_I2C_ADDRESS << 1)
#define DPS310_I2C_TIMING           0x00C4092AU
#define DPS310_I2C_TIMEOUT_MS       10U

#define DPS310_REG_PSR_B2           0x00U
#define DPS310_REG_PRS_CFG          0x06U
#define DPS310_REG_TMP_CFG          0x07U
#define DPS310_REG_MEAS_CFG         0x08U
#define DPS310_REG_CFG_REG          0x09U
#define DPS310_REG_RESET            0x0CU
#define DPS310_REG_ID               0x0DU
#define DPS310_REG_COEF             0x10U
#define DPS310_REG_COEF_SRCE        0x28U

#define DPS310_ID                   0x10U
#define SPL07_003_ID                0x11U
#define DPS310_ID_RETRY_COUNT       5U
#define DPS310_COEFF_COUNT          18U
#define SPL07_003_COEFF_COUNT       22U
#define DPS310_COEFF_CHUNK_SIZE     9U

#define DPS310_RESET_COMMAND        0x09U
#define DPS310_STATUS_COEF_READY    0x80U
#define DPS310_STATUS_SENSOR_READY  0x40U
#define DPS310_STATUS_TMP_READY     0x20U
#define DPS310_STATUS_PRS_READY     0x10U
#define DPS310_PRS_CFG_VALUE        0x54U
#define DPS310_TMP_CFG_VALUE        0x54U
#define DPS310_TMP_COEF_SOURCE      0x80U
#define DPS310_CFG_SHIFT_VALUE      0x0CU
#define DPS310_MEAS_CONTINUOUS      0x07U
#define DPS310_SCALE_FACTOR_16      253952.0f
#define DPS310_STANDARD_SEA_LEVEL_PA 101325.0f
#define DPS310_ALTITUDE_EXPONENT    0.190295f
#define DPS310_PRESSURE_ALPHA       0.15f
#define DPS310_TEMPERATURE_ALPHA    0.05f

I2C_HandleTypeDef hdps310_i2c;

static dps310_identity_t dps310_identity;
static dps310_calibration_t dps310_coefficients;
static uint8_t dps310_configured;
static float dps310_pressure_filtered;
static float dps310_temperature_filtered;
static float dps310_altitude_baseline;
static uint8_t dps310_filter_initialized;

static int32_t dps310_twos_complement(uint32_t value, uint8_t bits)
{
    if ((value & (1UL << (bits - 1U))) != 0U)
    {
        return (int32_t)value - (int32_t)(1UL << bits);
    }

    return (int32_t)value;
}

static uint8_t dps310_write_register(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hdps310_i2c,
                              DPS310_I2C_ADDRESS_8BIT,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              &value,
                              1U,
                              DPS310_I2C_TIMEOUT_MS) == HAL_OK)
        ? DPS310_OK : DPS310_CONFIG_FAIL;
}

static uint8_t dps310_read_registers(uint8_t reg,
                                     uint8_t *data,
                                     uint16_t length)
{
    if ((data == 0) || (length == 0U))
    {
        return DPS310_COEFF_READ_FAIL;
    }

    return (HAL_I2C_Mem_Read(&hdps310_i2c,
                             DPS310_I2C_ADDRESS_8BIT,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             length,
                             DPS310_I2C_TIMEOUT_MS) == HAL_OK)
        ? DPS310_OK : DPS310_COEFF_READ_FAIL;
}

static uint8_t dps310_read_coefficients(void)
{
    uint8_t coefficients[SPL07_003_COEFF_COUNT] = {0};
    uint16_t coefficient_count =
        (dps310_identity.is_spl07_003 != 0U)
        ? SPL07_003_COEFF_COUNT
        : DPS310_COEFF_COUNT;
    uint16_t offset;

    for (offset = 0U; offset < coefficient_count;
         offset = (uint16_t)(offset + DPS310_COEFF_CHUNK_SIZE))
    {
        uint16_t remaining =
            (uint16_t)(coefficient_count - offset);
        uint16_t chunk = (remaining > DPS310_COEFF_CHUNK_SIZE)
            ? DPS310_COEFF_CHUNK_SIZE
            : remaining;

        if (dps310_read_registers(
                (uint8_t)(DPS310_REG_COEF + offset),
                &coefficients[offset],
                chunk) != DPS310_OK)
        {
            return DPS310_COEFF_READ_FAIL;
        }
    }

    dps310_coefficients.c0 = (int16_t)dps310_twos_complement(
        ((uint32_t)coefficients[0] << 4) |
        (((uint32_t)coefficients[1] >> 4) & 0x0FU), 12U);
    dps310_coefficients.c1 = (int16_t)dps310_twos_complement(
        (((uint32_t)coefficients[1] & 0x0FU) << 8) |
        coefficients[2], 12U);
    dps310_coefficients.c00 = dps310_twos_complement(
        ((uint32_t)coefficients[3] << 12) |
        ((uint32_t)coefficients[4] << 4) |
        (((uint32_t)coefficients[5] >> 4) & 0x0FU), 20U);
    dps310_coefficients.c10 = dps310_twos_complement(
        (((uint32_t)coefficients[5] & 0x0FU) << 16) |
        ((uint32_t)coefficients[6] << 8) |
        coefficients[7], 20U);
    dps310_coefficients.c01 = (int16_t)dps310_twos_complement(
        ((uint32_t)coefficients[8] << 8) |
        coefficients[9], 16U);
    dps310_coefficients.c11 = (int16_t)dps310_twos_complement(
        ((uint32_t)coefficients[10] << 8) |
        coefficients[11], 16U);
    dps310_coefficients.c20 = (int16_t)dps310_twos_complement(
        ((uint32_t)coefficients[12] << 8) |
        coefficients[13], 16U);
    dps310_coefficients.c21 = (int16_t)dps310_twos_complement(
        ((uint32_t)coefficients[14] << 8) |
        coefficients[15], 16U);
    dps310_coefficients.c30 = (int16_t)dps310_twos_complement(
        ((uint32_t)coefficients[16] << 8) |
        coefficients[17], 16U);

    if (dps310_identity.is_spl07_003 != 0U)
    {
        dps310_coefficients.c31 = (int16_t)dps310_twos_complement(
            ((uint32_t)coefficients[18] << 4) |
            (((uint32_t)coefficients[19] >> 4) & 0x0FU), 12U);
        dps310_coefficients.c40 = (int16_t)dps310_twos_complement(
            (((uint32_t)coefficients[19] & 0x0FU) << 8) |
            coefficients[20], 12U);
    }
    else
    {
        dps310_coefficients.c31 = 0;
        dps310_coefficients.c40 = 0;
    }

    return DPS310_OK;
}

static uint8_t dps310_configure(void)
{
    uint8_t status = 0U;
    uint8_t temperature_config = DPS310_TMP_CFG_VALUE;
    uint8_t retry;

    if (dps310_write_register(DPS310_REG_RESET,
                              DPS310_RESET_COMMAND) != DPS310_OK)
    {
        return DPS310_RESET_FAIL;
    }

    HAL_Delay(40U);

    for (retry = 0U; retry < 10U; retry++)
    {
        if ((dps310_read_registers(DPS310_REG_MEAS_CFG,
                                   &status,
                                   1U) == DPS310_OK) &&
            ((status & (DPS310_STATUS_COEF_READY |
                        DPS310_STATUS_SENSOR_READY)) ==
             (DPS310_STATUS_COEF_READY |
              DPS310_STATUS_SENSOR_READY)))
        {
            break;
        }
        HAL_Delay(5U);
    }

    if (retry == 10U)
    {
        return DPS310_NOT_READY;
    }

    if (dps310_read_coefficients() != DPS310_OK)
    {
        return DPS310_COEFF_READ_FAIL;
    }

    if (dps310_identity.is_spl07_003 == 0U)
    {
        if (dps310_read_registers(DPS310_REG_COEF_SRCE,
                                  &temperature_config,
                                  1U) != DPS310_OK)
        {
            return DPS310_CONFIG_FAIL;
        }
        temperature_config =
            (uint8_t)((temperature_config & DPS310_TMP_COEF_SOURCE) |
                      DPS310_TMP_CFG_VALUE);
    }

    if ((dps310_write_register(DPS310_REG_PRS_CFG,
                               DPS310_PRS_CFG_VALUE) != DPS310_OK) ||
        (dps310_write_register(DPS310_REG_TMP_CFG,
                               temperature_config) != DPS310_OK) ||
        (dps310_write_register(DPS310_REG_CFG_REG,
                               DPS310_CFG_SHIFT_VALUE) != DPS310_OK) ||
        (dps310_write_register(DPS310_REG_MEAS_CFG,
                               DPS310_MEAS_CONTINUOUS) != DPS310_OK))
    {
        return DPS310_CONFIG_FAIL;
    }

    dps310_configured = 1U;
    return DPS310_OK;
}

/**
 * @brief 初始化 I2C1 的 PB6/PB7 引脚和时钟。
 * @param hi2c HAL I2C 句柄；仅处理 I2C1。
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    if (hi2c->Instance != I2C1)
    {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief 初始化 I2C1、识别 DPS310、读取系数并启动连续测量。
 * @retval DPS310_OK DPS310 或兼容 SPL07-003 已完成配置。
 * @retval DPS310_I2C_INIT_FAIL I2C1 初始化失败。
 * @retval DPS310_NOT_FOUND 多次读取均发生通信失败。
 * @retval DPS310_WRONG_ID 有应答，但芯片 ID 不是支持的型号。
 * @retval DPS310_RESET_FAIL 软件复位失败。
 * @retval DPS310_NOT_READY 系数或传感器未进入就绪状态。
 * @retval DPS310_COEFF_READ_FAIL 校准系数读取失败。
 * @retval DPS310_CONFIG_FAIL 测量配置写入失败。
 */
uint8_t dps310_init(void)
{
    uint8_t identity = 0U;
    uint8_t retry;
    HAL_StatusTypeDef read_status = HAL_ERROR;

    hdps310_i2c.Instance = I2C1;
    hdps310_i2c.Init.Timing = DPS310_I2C_TIMING;
    hdps310_i2c.Init.OwnAddress1 = 0U;
    hdps310_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hdps310_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hdps310_i2c.Init.OwnAddress2 = 0U;
    hdps310_i2c.Init.OwnAddress2Masks = 0U;
    hdps310_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hdps310_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hdps310_i2c) != HAL_OK)
    {
        return DPS310_I2C_INIT_FAIL;
    }

    (void)HAL_I2CEx_ConfigAnalogFilter(&hdps310_i2c,
                                       I2C_ANALOGFILTER_ENABLE);

    for (retry = 0U; retry < DPS310_ID_RETRY_COUNT; retry++)
    {
        HAL_Delay(100U);
        read_status = HAL_I2C_Mem_Read(&hdps310_i2c,
                                       DPS310_I2C_ADDRESS_8BIT,
                                       DPS310_REG_ID,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &identity,
                                       1U,
                                       DPS310_I2C_TIMEOUT_MS);
        if (read_status == HAL_OK)
        {
            break;
        }
    }

    dps310_identity.identity = identity;
    dps310_identity.is_spl07_003 =
        (identity == SPL07_003_ID) ? 1U : 0U;

    if (read_status != HAL_OK)
    {
        return DPS310_NOT_FOUND;
    }

    if ((identity != DPS310_ID) && (identity != SPL07_003_ID))
    {
        return DPS310_WRONG_ID;
    }

    return dps310_configure();
}

/**
 * @brief 读取一次 DPS310 原始数据并执行数据手册补偿计算。
 * @param measurement 原始值、补偿后的气压和温度输出。
 * @retval DPS310_OK 读取和计算成功。
 * @retval DPS310_NO_DATA 温度和气压尚未同时就绪。
 * @retval DPS310_READ_FAIL I2C 读取失败或参数为空。
 */
uint8_t dps310_read_measurement(dps310_measurement_t *measurement)
{
    uint8_t status;
    uint8_t data[6];
    float pressure_scaled;
    float temperature_scaled;
    float c00;
    float c01;
    float c10;
    float c11;
    float c20;
    float c21;
    float c30;
    float c31;
    float c40;
    float c0;
    float c1;

    if ((measurement == 0) || (dps310_configured == 0U))
    {
        return DPS310_READ_FAIL;
    }

    if (dps310_read_registers(DPS310_REG_MEAS_CFG,
                              &status,
                              1U) != DPS310_OK)
    {
        return DPS310_READ_FAIL;
    }

    if ((status & (DPS310_STATUS_TMP_READY |
                   DPS310_STATUS_PRS_READY)) !=
        (DPS310_STATUS_TMP_READY | DPS310_STATUS_PRS_READY))
    {
        return DPS310_NO_DATA;
    }

    if (dps310_read_registers(DPS310_REG_PSR_B2,
                              data,
                              sizeof(data)) != DPS310_OK)
    {
        return DPS310_READ_FAIL;
    }

    measurement->raw_pressure = dps310_twos_complement(
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        data[2], 24U);
    measurement->raw_temperature = dps310_twos_complement(
        ((uint32_t)data[3] << 16) |
        ((uint32_t)data[4] << 8) |
        data[5], 24U);

    pressure_scaled =
        (float)measurement->raw_pressure / DPS310_SCALE_FACTOR_16;
    temperature_scaled =
        (float)measurement->raw_temperature / DPS310_SCALE_FACTOR_16;

    c00 = (float)dps310_coefficients.c00;
    c01 = (float)dps310_coefficients.c01;
    c10 = (float)dps310_coefficients.c10;
    c11 = (float)dps310_coefficients.c11;
    c20 = (float)dps310_coefficients.c20;
    c21 = (float)dps310_coefficients.c21;
    c30 = (float)dps310_coefficients.c30;
    c31 = (float)dps310_coefficients.c31;
    c40 = (float)dps310_coefficients.c40;
    c0 = (float)dps310_coefficients.c0;
    c1 = (float)dps310_coefficients.c1;

    if (dps310_identity.is_spl07_003 != 0U)
    {
        measurement->pressure_pa =
            c00 +
            pressure_scaled *
                (c10 + pressure_scaled *
                    (c20 + pressure_scaled *
                        (c30 + pressure_scaled * c40))) +
            temperature_scaled * c01 +
            temperature_scaled * pressure_scaled *
                (c11 + pressure_scaled *
                    (c21 + pressure_scaled * c31));
    }
    else
    {
        measurement->pressure_pa =
            c00 +
            pressure_scaled *
                (c10 + pressure_scaled *
                    (c20 + pressure_scaled * c30)) +
            temperature_scaled * c01 +
            temperature_scaled * pressure_scaled *
                (c11 + pressure_scaled * c21);
    }

    measurement->temperature_c =
        (c0 * 0.5f) + (c1 * temperature_scaled);
    return DPS310_OK;
}

/**
 * @brief 读取并滤波气压/温度，并计算相对首次有效气压的高度。
 * @param measurement 滤波后的气压、温度和相对高度输出。
 * @retval DPS310_OK 数据读取和计算成功。
 * @retval DPS310_NO_DATA 数据尚未就绪。
 * @retval DPS310_READ_FAIL I2C 读取失败或参数为空。
 * @note 气压滤波系数为 0.15，温度滤波系数为 0.05；
 *       相对高度以启动后第一个有效气压为 0 m。
 */
uint8_t dps310_read_processed(dps310_processed_t *measurement)
{
    dps310_measurement_t raw;
    float absolute_altitude;

    if (measurement == 0)
    {
        return DPS310_READ_FAIL;
    }

    if (dps310_read_measurement(&raw) != DPS310_OK)
    {
        return DPS310_NO_DATA;
    }

    if (dps310_filter_initialized == 0U)
    {
        dps310_pressure_filtered = raw.pressure_pa;
        dps310_temperature_filtered = raw.temperature_c;
        dps310_altitude_baseline = raw.pressure_pa;
        dps310_filter_initialized = 1U;
    }
    else
    {
        dps310_pressure_filtered +=
            DPS310_PRESSURE_ALPHA *
            (raw.pressure_pa - dps310_pressure_filtered);
        dps310_temperature_filtered +=
            DPS310_TEMPERATURE_ALPHA *
            (raw.temperature_c - dps310_temperature_filtered);
    }

    if ((dps310_pressure_filtered > 1000.0f) &&
        (dps310_altitude_baseline > 1000.0f))
    {
        absolute_altitude = 44330.0f *
            (1.0f - powf(
                dps310_pressure_filtered / DPS310_STANDARD_SEA_LEVEL_PA,
                DPS310_ALTITUDE_EXPONENT));
        measurement->altitude_relative_m =
            absolute_altitude -
            44330.0f *
                (1.0f - powf(
                    dps310_altitude_baseline /
                        DPS310_STANDARD_SEA_LEVEL_PA,
                    DPS310_ALTITUDE_EXPONENT));
    }
    else
    {
        measurement->altitude_relative_m = 0.0f;
    }

    measurement->pressure_pa = dps310_pressure_filtered;
    measurement->temperature_c = dps310_temperature_filtered;
    return DPS310_OK;
}

/**
 * @brief 复制最近一次检测到的芯片 ID 和兼容型号标志。
 * @param identity 型号信息输出；为空时直接返回。
 */
void dps310_get_identity(dps310_identity_t *identity)
{
    if (identity == 0)
    {
        return;
    }

    identity->identity = dps310_identity.identity;
    identity->is_spl07_003 = dps310_identity.is_spl07_003;
}

/**
 * @brief 复制最近一次读取并解析后的 DPS310 校准系数。
 * @param calibration 校准系数输出；为空时直接返回。
 */
void dps310_get_calibration(dps310_calibration_t *calibration)
{
    if (calibration == 0)
    {
        return;
    }

    *calibration = dps310_coefficients;
}

/**
 * @brief 查询 DPS310 是否已成功进入连续测量模式。
 * @return 1 表示已配置，0 表示尚未配置。
 */
uint8_t dps310_is_configured(void)
{
    return dps310_configured;
}
