#ifndef GYRO_SENSOR_H
#define GYRO_SENSOR_H

#include "stm32f7xx_hal.h"

#define GYRO_SENSOR_OK            0U
#define GYRO_SENSOR_SPI_INIT_FAIL 1U
#define GYRO_SENSOR_READ_FAIL     2U
#define GYRO_SENSOR_WRONG_ID      3U
#define GYRO_SENSOR_RESET_FAIL    4U
#define GYRO_SENSOR_CONFIG_FAIL   5U
#define GYRO_SENSOR_DMA_BUSY      6U
#define GYRO_SENSOR_NO_DATA       7U

#define GYRO_FIFO_WATERMARK       16U
#define GYRO_FIFO_MAX_RECORDS     32U

#define GYRO_PROBE_COUNT          4U
#define GYRO_PROBE_MODE3_FAST     0U
#define GYRO_PROBE_MODE3_SLOW     1U
#define GYRO_PROBE_MODE0_FAST     2U
#define GYRO_PROBE_MODE0_SLOW     3U

typedef struct
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} gyro_sensor_sample_t;

typedef struct
{
    uint16_t gyro_count;
    uint8_t accel_valid;
    gyro_sensor_sample_t gyro_samples[GYRO_FIFO_MAX_RECORDS];
    gyro_sensor_sample_t accel_sample;
} gyro_sensor_fifo_batch_t;

/** @brief 用四组 SPI 参数探测传感器 WHO_AM_I。 */
uint8_t gyro_sensor_probe(uint8_t identities[GYRO_PROBE_COUNT]);

/** @brief 识别、复位并配置陀螺仪和加速度计。 */
uint8_t gyro_sensor_init(uint8_t *identity);

/** @brief 配置 FIFO 水位并进入连续 FIFO 模式。 */
uint8_t gyro_sensor_enable_fifo(void);

/** @brief 清除残留 DMA/EXTI 状态并重新同步连续 FIFO。 */
uint8_t gyro_sensor_recover_fifo(void);

/** @brief 阻塞读取一组陀螺仪和加速度计原始样本。 */
uint8_t gyro_sensor_read(gyro_sensor_sample_t *sample);

/** @brief 启动一次普通寄存器区的 DMA 读取。 */
uint8_t gyro_sensor_start_dma_read(void);

/** @brief 完成普通寄存器区 DMA 读取并解析样本。 */
uint8_t gyro_sensor_finish_dma_read(gyro_sensor_sample_t *sample);

/** @brief 查询 FIFO 长度并启动带标签数据的 DMA 读取。 */
uint8_t gyro_sensor_start_fifo_dma_read(void);

/** @brief 完成 FIFO DMA 读取并解析一批传感器记录。 */
uint8_t gyro_sensor_finish_fifo_dma_read(gyro_sensor_fifo_batch_t *batch);

/** @brief 中止当前传感器 DMA 读取。 */
void gyro_sensor_abort_dma_read(void);

/** @brief 原子读取并清除 FIFO 数据就绪标志。 */
uint8_t gyro_sensor_take_data_ready(void);

/** @brief 将陀螺仪原始值换算为百分之一度每秒。 */
int32_t gyro_sensor_raw_to_centidps(int16_t raw);

/** @brief 将加速度计原始值换算为百分之一重力加速度。 */
int32_t gyro_sensor_raw_to_centig(int16_t raw);

#endif
