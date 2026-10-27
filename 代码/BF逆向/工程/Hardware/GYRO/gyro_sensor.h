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

uint8_t gyro_sensor_probe(uint8_t identities[GYRO_PROBE_COUNT]);
uint8_t gyro_sensor_init(uint8_t *identity);
uint8_t gyro_sensor_enable_fifo(void);
uint8_t gyro_sensor_read(gyro_sensor_sample_t *sample);
uint8_t gyro_sensor_start_dma_read(void);
uint8_t gyro_sensor_finish_dma_read(gyro_sensor_sample_t *sample);
uint8_t gyro_sensor_start_fifo_dma_read(void);
uint8_t gyro_sensor_finish_fifo_dma_read(gyro_sensor_fifo_batch_t *batch);
void gyro_sensor_abort_dma_read(void);
uint8_t gyro_sensor_take_data_ready(void);
int32_t gyro_sensor_raw_to_centidps(int16_t raw);
int32_t gyro_sensor_raw_to_centig(int16_t raw);

#endif
