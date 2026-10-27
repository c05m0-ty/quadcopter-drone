#ifndef GYRO_ATTITUDE_H
#define GYRO_ATTITUDE_H

#include "gyro_sensor.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} gyro_attitude_t;

void gyro_attitude_reset(void);
void gyro_attitude_zero(void);
void gyro_attitude_update(const gyro_sensor_sample_t *sample,
                          uint32_t delta_time_ms,
                          gyro_attitude_t *attitude);

#endif
