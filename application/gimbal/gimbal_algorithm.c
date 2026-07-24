#include "gimbal_algorithm.h"
#include "gimbal.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "arm_math.h"

#define YAW_SPEED_FF_K                      1.2f                     // Yaw轴速度前馈系数
#define PITCH_GRAVITY_FF_K                  1.0f                     // Pitch轴重力补偿前馈系数
#define PITCH_GRAVITY_FF_FUCTION_A          0.0f                     // 重力补偿函数的A * cosf(beta) + B * sinf(beta)
#define PITCH_GRAVITY_FF_FUCTION_B          0.0f                     // 重力补偿函数的A * cosf(beta) + B * sinf(beta)


static void FloatLimit(float *value, float min, float max)
{
    if (*value < min)
    {
        *value = min;
    }
    else if (*value > max)
    {
        *value = max;
    }
}

/**
 * @brief Yaw轴轨迹规划函数
 *
 * @param state Yaw 轴前馈控制的状态结构体 
 * 
 * @param target_yaw 遥控器目标Yaw轴角度
 * 
 */
static void YawTargetStateUpdate(YawFeedforwardState_t *state, float target_yaw)
{
    state->dt = DWT_GetDeltaT(&state->last_tick);
    state->valid = 0;

    if (!state->initialized)
    {
        state->initialized = 1;
        state->last_target_yaw = target_yaw;

        state->target_yaw_speed = 0.0f;
        state->target_yaw_speed_lpf = 0.0f;
        state->last_target_yaw_speed_lpf = 0.0f;
        state->target_yaw_acc = 0.0f;
        state->target_yaw_acc_lpf = 0.0f;
        return;
    }

    if (state->dt <= 0.0001f || state->dt > 0.02f)
    {
        state->last_target_yaw = target_yaw;

        state->target_yaw_speed = 0.0f;
        state->target_yaw_speed_lpf = 0.0f;
        state->last_target_yaw_speed_lpf = 0.0f;
        state->target_yaw_acc = 0.0f;
        state->target_yaw_acc_lpf = 0.0f;
        return;
    }

    state->target_yaw_speed = (target_yaw - state->last_target_yaw) / state->dt;

    float speed_alpha = state->dt / (YAW_TARGET_SPEED_LPF_TAU + state->dt);
    state->target_yaw_speed_lpf +=
        speed_alpha * (state->target_yaw_speed - state->target_yaw_speed_lpf);

    state->target_yaw_acc =
        (state->target_yaw_speed_lpf - state->last_target_yaw_speed_lpf) / state->dt;

    float acc_alpha = state->dt / (YAW_TARGET_ACC_LPF_TAU + state->dt);
    state->target_yaw_acc_lpf +=
        acc_alpha * (state->target_yaw_acc - state->target_yaw_acc_lpf);

    state->last_target_yaw = target_yaw;
    state->last_target_yaw_speed_lpf = state->target_yaw_speed_lpf;
    state->valid = 1;
}


/**
 * @brief Yaw轴速度前馈
 *
 * @param state Yaw 轴前馈控制的状态结构体 
 * 
 */
static void YawSpeedFeedforwardUpdate(YawFeedforwardState_t *state)
{
    if (!state->valid)
    {
        gimbal_ff.yaw_speed_ff = 0.0f;
        return;
    }

    gimbal_ff.yaw_speed_ff = YAW_SPEED_FF_K * state->target_yaw_speed_lpf;
    FloatLimit(&gimbal_ff.yaw_speed_ff, -YAW_SPEED_FF_MAX, YAW_SPEED_FF_MAX);
}


/**
 * @brief Yaw轴加速度前馈
 *
 * @param state Yaw 轴前馈控制的状态结构体 
 * 
 */
static void YawCurrentFeedforwardUpdate(YawFeedforwardState_t *state)
{
    if (!state->valid)
    {
        gimbal_ff.yaw_current_ff = 0.0f;
        return;
    }

    gimbal_ff.yaw_current_ff = YAW_CURRENT_FF_K * state->target_yaw_acc_lpf;
    FloatLimit(&gimbal_ff.yaw_current_ff, -YAW_CURRENT_FF_MAX, YAW_CURRENT_FF_MAX);
}

/**
 * @brief Yaw轴加速度前馈
 *
 * @param target_yaw 遥控器目标Yaw轴角度
 * 
 */
void YawFeedforwardUpdate(float target_yaw)
{
    static YawFeedforwardState_t yaw_ff_state = {0};

    YawTargetStateUpdate(&yaw_ff_state, target_yaw);
    YawSpeedFeedforwardUpdate(&yaw_ff_state);
    YawCurrentFeedforwardUpdate(&yaw_ff_state);
}


// /**
//  * @brief Yaw轴速度前馈
//  *
//  * @param target_yaw 遥控器的控制角度
//  * 
//  */
// static float YawSpeedFeedforwardUpdate(float target_yaw)
// {
//     static float last_target_yaw = 0.0f;
//     static float target_yaw_speed = 0.0f;
//     static uint32_t last_tick = 0;
//     static uint8_t initialized = 0;

//     float dt = DWT_GetDeltaT(&last_tick);

//     if (!initialized)
//     {
//         initialized = 1;
//         last_target_yaw = target_yaw;
//         target_yaw_speed = 0.0f;
//         gimbal_ff.yaw_speed_ff = 0.0f;
//         return gimbal_ff.yaw_speed_ff;
//     }

//     if (dt > 0.0001f && dt < 0.02f)
//     {
//         target_yaw_speed = (target_yaw - last_target_yaw) / dt;

//         gimbal_ff.yaw_speed_ff = YAW_SPEED_FF_K * target_yaw_speed;
//         LIMIT_MIN_MAX(gimbal_ff.yaw_speed_ff, -300.0f, 300.0f);
//     }
//     else
//     {
//         target_yaw_speed = 0.0f;
//         gimbal_ff.yaw_speed_ff = 0.0f;
//     }

//     last_target_yaw = target_yaw;

//     return gimbal_ff.yaw_speed_ff;
// }


/**
 * @brief Pitch轴重力补偿
 *
 * @param  actual_pitch 实际pitch轴角度
 * 
 */
float PitchGravityFeedforwardUpdate(const float actual_pitch)
{
    float beta = actual_pitch * PI / 180.0f;
    
    gimbal_ff.pitch_current_ff = PITCH_GRAVITY_FF_FUCTION_A * cosf(beta) + PITCH_GRAVITY_FF_FUCTION_B * sinf(beta);

    return  0.38; // PITCH_GRAVITY_FF_K * pitch_gravity_feedforward;
}


// static void GimbalStabilityCalc(const attitude_t *gimbal_imu_data, const attitude_t *chassis_imu_data, GimbalComp_t *gimbal_comp)
// {
    
//     float fb_yaw = gimbal_imu_data->Gyro[2];   // Z轴 → yaw角速度

//     float fb_pitch = gimbal_imu_data->Gyro[1];   // Y轴 → pitch角速度


//     float yaw_rad = chassis_imu_data->Yaw * PI / 180.0f;

//     float ff_yaw = -chassis_imu_data->Gyro[2];

//     float ff_pitch = sinf(yaw_rad) * chassis_imu_data->Gyro[0] - cosf(yaw_rad) * chassis_imu_data->Gyro[1];

//     gimbal_comp->yaw_speed = fb_yaw;// + ff_yaw;

//     gimbal_comp->pitch_speed = fb_pitch;// + ff_pitch;

// }
