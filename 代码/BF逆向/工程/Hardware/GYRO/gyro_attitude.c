#include "gyro_attitude.h"
#include <math.h>

#define GYRO_PI                    3.14159265358979323846f
#define GYRO_DEG_TO_RAD            (GYRO_PI / 180.0f)
#define GYRO_ACCEL_G_PER_LSB       (1.0f / 2048.0f)
#define GYRO_MAHONY_KP             0.60f
#define GYRO_MAHONY_KI             0.02f
#define GYRO_USE_ACCEL_CORRECTION  1U

static float quaternion_w = 1.0f;
static float quaternion_x;
static float quaternion_y;
static float quaternion_z;
static float zero_w = 1.0f;
static float zero_x;
static float zero_y;
static float zero_z;
static float integral_x;
static float integral_y;
static float integral_z;
static uint8_t attitude_initialized;

/**
 * @brief 归一化全局姿态四元数；异常零向量回退为单位四元数。
 */
static void quaternion_normalize(void)
{
    float norm = sqrtf((quaternion_w * quaternion_w) +
                       (quaternion_x * quaternion_x) +
                       (quaternion_y * quaternion_y) +
                       (quaternion_z * quaternion_z));

    if (norm > 0.000001f)
    {
        quaternion_w /= norm;
        quaternion_x /= norm;
        quaternion_y /= norm;
        quaternion_z /= norm;
    }
    else
    {
        quaternion_w = 1.0f;
        quaternion_x = 0.0f;
        quaternion_y = 0.0f;
        quaternion_z = 0.0f;
    }
}

/**
 * @brief 计算两个四元数的哈密顿乘积。
 * @param aw 四元数 A 的实部。
 * @param ax 四元数 A 的 X 分量。
 * @param ay 四元数 A 的 Y 分量。
 * @param az 四元数 A 的 Z 分量。
 * @param bw 四元数 B 的实部。
 * @param bx 四元数 B 的 X 分量。
 * @param by 四元数 B 的 Y 分量。
 * @param bz 四元数 B 的 Z 分量。
 * @param result 乘积 A * B 的输出。
 */
static void quaternion_multiply(float aw, float ax, float ay, float az,
                                float bw, float bx, float by, float bz,
                                gyro_attitude_t *result)
{
    result->w = (aw * bw) - (ax * bx) - (ay * by) - (az * bz);
    result->x = (aw * bx) + (ax * bw) + (ay * bz) - (az * by);
    result->y = (aw * by) - (ax * bz) + (ay * bw) + (az * bx);
    result->z = (aw * bz) + (ax * by) - (ay * bx) + (az * bw);
}

/**
 * @brief 将当前姿态的共轭保存为零点四元数。
 * @note 当前坐标系中的姿态右乘该共轭后，可得到相对零点的姿态。
 */
static void quaternion_set_zero_from_current(void)
{
    zero_w = quaternion_w;
    zero_x = -quaternion_x;
    zero_y = -quaternion_y;
    zero_z = -quaternion_z;
}

/**
 * @brief 计算当前绝对姿态相对零点姿态的四元数。
 * @param attitude 相对姿态输出。
 */
static void quaternion_get_relative(gyro_attitude_t *attitude)
{
    quaternion_multiply(quaternion_w, quaternion_x,
                        quaternion_y, quaternion_z,
                        zero_w, zero_x, zero_y, zero_z,
                        attitude);
}

/**
 * @brief 根据加速度计重力方向估算初始 Roll/Pitch，并建立零点基准。
 * @param sample 包含加速度计原始值的传感器样本。
 * @note Yaw 没有磁力计绝对参考，因此初始 Yaw 视为零。
 */
static void quaternion_initialize_from_accel(
    const gyro_sensor_sample_t *sample)
{
    float ax = (float)sample->accel_x * GYRO_ACCEL_G_PER_LSB;
    float ay = (float)sample->accel_y * GYRO_ACCEL_G_PER_LSB;
    float az = (float)sample->accel_z * GYRO_ACCEL_G_PER_LSB;
    float roll = 0.0f;
    float pitch = 0.0f;
    float cr;
    float sr;
    float cp;
    float sp;

    if ((ax * ax) + (ay * ay) + (az * az) > 0.04f)
    {
        roll = atan2f(ay, az);
        pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
    }

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    quaternion_w = cr * cp;
    quaternion_x = sr * cp;
    quaternion_y = cr * sp;
    quaternion_z = -sr * sp;
    quaternion_normalize();

    integral_x = 0.0f;
    integral_y = 0.0f;
    integral_z = 0.0f;
    attitude_initialized = 1U;
    quaternion_set_zero_from_current();
}

/**
 * @brief 清除姿态、积分器和零点，恢复为未初始化的单位四元数状态。
 */
void gyro_attitude_reset(void)
{
    quaternion_w = 1.0f;
    quaternion_x = 0.0f;
    quaternion_y = 0.0f;
    quaternion_z = 0.0f;
    zero_w = 1.0f;
    zero_x = 0.0f;
    zero_y = 0.0f;
    zero_z = 0.0f;
    integral_x = 0.0f;
    integral_y = 0.0f;
    integral_z = 0.0f;
    attitude_initialized = 0U;
}

/**
 * @brief 将当前姿态设为新的显示零点，不改变内部绝对姿态。
 */
void gyro_attitude_zero(void)
{
    quaternion_set_zero_from_current();
}

/**
 * @brief 用一组陀螺仪/加速度计样本执行 Mahony 姿态更新。
 * @param sample 传感器原始样本。
 * @param delta_time_ms 本样本对应的积分时间，单位为毫秒。
 * @param attitude 相对零点的四元数姿态输出。
 * @note 首次调用使用加速度计确定 Roll/Pitch；之后陀螺仪积分并由
 *       加速度计重力方向修正漂移。时间步长会被限制在 1 至 200 ms。
 */
void gyro_attitude_update(const gyro_sensor_sample_t *sample,
                          uint32_t delta_time_ms,
                          gyro_attitude_t *attitude)
{
    float delta_time;
    float gyro_x;
    float gyro_y;
    float gyro_z;

    if ((sample == 0) || (attitude == 0))
    {
        return;
    }

    if (attitude_initialized == 0U)
    {
        quaternion_initialize_from_accel(sample);
        quaternion_get_relative(attitude);
        return;
    }

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

#if GYRO_USE_ACCEL_CORRECTION
    {
        float ax = (float)sample->accel_x * GYRO_ACCEL_G_PER_LSB;
        float ay = (float)sample->accel_y * GYRO_ACCEL_G_PER_LSB;
        float az = (float)sample->accel_z * GYRO_ACCEL_G_PER_LSB;
        float accel_norm = sqrtf((ax * ax) + (ay * ay) + (az * az));

        if (accel_norm > 0.2f)
        {
            float gravity_x;
            float gravity_y;
            float gravity_z;
            float error_x;
            float error_y;
            float error_z;

            ax /= accel_norm;
            ay /= accel_norm;
            az /= accel_norm;

            gravity_x = 2.0f *
                ((quaternion_x * quaternion_z) -
                 (quaternion_w * quaternion_y));
            gravity_y = 2.0f *
                ((quaternion_w * quaternion_x) +
                 (quaternion_y * quaternion_z));
            gravity_z = (quaternion_w * quaternion_w) -
                        (quaternion_x * quaternion_x) -
                        (quaternion_y * quaternion_y) +
                        (quaternion_z * quaternion_z);

            error_x = (ay * gravity_z) - (az * gravity_y);
            error_y = (az * gravity_x) - (ax * gravity_z);
            error_z = (ax * gravity_y) - (ay * gravity_x);

            integral_x += GYRO_MAHONY_KI * error_x * delta_time;
            integral_y += GYRO_MAHONY_KI * error_y * delta_time;
            integral_z += GYRO_MAHONY_KI * error_z * delta_time;

            gyro_x += (GYRO_MAHONY_KP * error_x) + integral_x;
            gyro_y += (GYRO_MAHONY_KP * error_y) + integral_y;
            gyro_z += (GYRO_MAHONY_KP * error_z) + integral_z;
        }
    }
#endif

    {
        float half_dt = 0.5f * delta_time;
        float old_w = quaternion_w;
        float old_x = quaternion_x;
        float old_y = quaternion_y;
        float old_z = quaternion_z;

        quaternion_w += half_dt *
            (-(old_x * gyro_x) -
              (old_y * gyro_y) -
              (old_z * gyro_z));
        quaternion_x += half_dt *
            ((old_w * gyro_x) +
             (old_y * gyro_z) -
             (old_z * gyro_y));
        quaternion_y += half_dt *
            ((old_w * gyro_y) -
             (old_x * gyro_z) +
             (old_z * gyro_x));
        quaternion_z += half_dt *
            ((old_w * gyro_z) +
             (old_x * gyro_y) -
             (old_y * gyro_x));
        quaternion_normalize();
    }

    quaternion_get_relative(attitude);
}
