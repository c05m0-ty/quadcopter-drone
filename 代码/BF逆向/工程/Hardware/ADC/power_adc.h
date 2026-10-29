#ifndef POWER_ADC_H
#define POWER_ADC_H

#include "stm32f7xx_hal.h"

#define POWER_ADC_OK                 0U
#define POWER_ADC_BAD_PARAM          1U
#define POWER_ADC_DMA_INIT_FAIL      2U
#define POWER_ADC_INIT_FAIL          3U
#define POWER_ADC_CHANNEL_FAIL       4U
#define POWER_ADC_START_FAIL         5U

typedef struct
{
    uint16_t current_raw;
    uint16_t voltage_raw;
} power_adc_raw_t;

typedef struct
{
    uint16_t current_raw;
    uint16_t voltage_raw;
    float voltage_v;
    float current_a;
} power_measurement_t;

/** @brief 初始化 PC1 电流和 PC2 电压的 ADC3 连续扫描/DMA 采样。 */
uint8_t power_adc_init(void);

/** @brief 取得最近一次电流和电池电压 ADC 原始值。 */
void power_adc_get_raw(power_adc_raw_t *sample);

/** @brief 取得经过校准和低通滤波的电压、电流测量值。 */
void power_adc_get_measurement(power_measurement_t *sample);

#endif
