#include "gyro_attitude.h"
#include <math.h>

#define GYRO_PI                    3.14159265358979323846f
#define GYRO_DEG_TO_RAD            (GYRO_PI / 180.0f)
#define GYRO_ACCEL_G_PER_LSB       (1.0f / 2048.0f)
#define GYRO_COMPLEMENTARY_WEIGHT  0.98f

static float roll_angle;
static float pitch_angle;
static float yaw_angle;
static float roll_offset;
static float pitch_offset;
static float yaw_offset;
static uint8_t attitude_initialized;

static float wrap_angle(float angle)
{
    while (angle > GYRO_PI)
    {
        angle -= (2.0f * GYRO_PI);
    }
    while (angle < -GYRO_PI)
    {
        angle += (2.0f * GYRO_PI);
    }
    return angle;
}

void gyro_attitude_reset(void)
{
    roll_angle = 0.0f;
    pitch_angle = 0.0f;
    yaw_angle = 0.0f;
    roll_offset = 0.0f;
    pitch_offset = 0.0f;
    yaw_offset = 0.0f;
    attitude_initialized = 0U;
}

void gyro_attitude_zero(void)
{
    roll_offset = roll_angle;
    pitch_offset = pitch_angle;
    yaw_offset = yaw_angle;
}

void gyro_attitude_update(const gyro_sensor_sample_t *sample,
                          uint32_t delta_time_ms,
                          gyro_attitude_t *attitude)
{
    float ax;
    float ay;
    float az;
    float accel_horizontal;
    float accel_norm_squared;
    float accel_roll;
    float accel_pitch;
    float delta_time;
    float gyro_x;
    float gyro_y;
    float gyro_z;

    if ((sample == 0) || (attitude == 0))
    {
        return;
    }

    ax = (float)sample->accel_x * GYRO_ACCEL_G_PER_LSB;
    ay = (float)sample->accel_y * GYRO_ACCEL_G_PER_LSB;
    az = (float)sample->accel_z * GYRO_ACCEL_G_PER_LSB;
    accel_horizontal = sqrtf((ay * ay) + (az * az));
    accel_norm_squared =
        (ax * ax) + (ay * ay) + (az * az);

    accel_roll = atan2f(ay, az);
    accel_pitch = atan2f(-ax, accel_horizontal);

    if (attitude_initialized == 0U)
    {
        roll_angle = (accel_norm_squared > 0.04f)
            ? accel_roll : 0.0f;
        pitch_angle = (accel_norm_squared > 0.04f)
            ? accel_pitch : 0.0f;
        yaw_angle = 0.0f;
        roll_offset = roll_angle;
        pitch_offset = pitch_angle;
        yaw_offset = yaw_angle;
        attitude_initialized = 1U;
    }
    else
    {
        if (delta_time_ms == 0U)
        {
            delta_time_ms = 1U;
        }
        else if (delta_time_ms > 200U)
        {
            delta_time_ms = 200U;
        }

        delta_time = (float)delta_time_ms * 0.001f;
        gyro_x = (float)sample->gyro_x * 0.07f * GYRO_DEG_TO_RAD;
        gyro_y = (float)sample->gyro_y * 0.07f * GYRO_DEG_TO_RAD;
        gyro_z = (float)sample->gyro_z * 0.07f * GYRO_DEG_TO_RAD;

        roll_angle += gyro_x * delta_time;
        pitch_angle += gyro_y * delta_time;
        yaw_angle += gyro_z * delta_time;

        if (accel_norm_squared > 0.04f)
        {
            roll_angle = (GYRO_COMPLEMENTARY_WEIGHT * roll_angle) +
                         ((1.0f - GYRO_COMPLEMENTARY_WEIGHT) * accel_roll);
            pitch_angle = (GYRO_COMPLEMENTARY_WEIGHT * pitch_angle) +
                          ((1.0f - GYRO_COMPLEMENTARY_WEIGHT) * accel_pitch);
        }
        yaw_angle = wrap_angle(yaw_angle);
    }

    attitude->roll = roll_angle - roll_offset;
    attitude->pitch = pitch_angle - pitch_offset;
    attitude->yaw = wrap_angle(yaw_angle - yaw_offset);
}
