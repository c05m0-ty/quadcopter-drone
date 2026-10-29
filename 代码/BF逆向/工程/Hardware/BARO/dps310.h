#ifndef DPS310_H
#define DPS310_H

#include "stm32f7xx_hal.h"

#define DPS310_OK                  0U
#define DPS310_I2C_INIT_FAIL       1U
#define DPS310_NOT_FOUND           2U
#define DPS310_WRONG_ID            3U
#define DPS310_RESET_FAIL          4U
#define DPS310_NOT_READY           5U
#define DPS310_COEFF_READ_FAIL     6U
#define DPS310_CONFIG_FAIL         7U
#define DPS310_NO_DATA             8U
#define DPS310_READ_FAIL           9U

typedef struct
{
    uint8_t identity;
    uint8_t is_spl07_003;
} dps310_identity_t;

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    int16_t c31;
    int16_t c40;
} dps310_calibration_t;

typedef struct
{
    int32_t raw_pressure;
    int32_t raw_temperature;
    float pressure_pa;
    float temperature_c;
} dps310_measurement_t;

typedef struct
{
    float pressure_pa;
    float temperature_c;
    float altitude_relative_m;
} dps310_processed_t;

/** @brief 初始化 I2C1，并检测板载 DPS310 气压计。 */
uint8_t dps310_init(void);

/** @brief 读取一次原始气压/温度并计算补偿后的实际值。 */
uint8_t dps310_read_measurement(dps310_measurement_t *measurement);

/** @brief 读取并滤波气压/温度，同时计算相对启动点的高度。 */
uint8_t dps310_read_processed(dps310_processed_t *measurement);

/** @brief 获取最近一次检测到的芯片型号信息。 */
void dps310_get_identity(dps310_identity_t *identity);

/** @brief 复制芯片内部读取并解析后的校准系数。 */
void dps310_get_calibration(dps310_calibration_t *calibration);

/** @brief 查询 DPS310 是否已进入温度和气压连续测量。 */
uint8_t dps310_is_configured(void);

#endif
