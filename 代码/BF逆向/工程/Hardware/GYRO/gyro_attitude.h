#ifndef GYRO_ATTITUDE_H
#define GYRO_ATTITUDE_H

#include "gyro_sensor.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} gyro_attitude_t;

/** @brief 清除姿态状态并等待下一组样本重新初始化。 */
void gyro_attitude_reset(void);

/** @brief 将当前姿态设置为新的相对零点。 */
void gyro_attitude_zero(void);

/** @brief 使用一组传感器样本更新 Mahony 四元数姿态。 */
void gyro_attitude_update(const gyro_sensor_sample_t *sample,
                          uint32_t delta_time_ms,
                          gyro_attitude_t *attitude);

#endif
