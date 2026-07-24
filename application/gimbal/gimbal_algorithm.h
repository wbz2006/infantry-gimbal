// gimbal_algorithm.h
#ifndef GIMBAL_ALGORITHM_H
#define GIMBAL_ALGORITHM_H

#include "stdint.h"
#include "ins_task.h"

#define YAW_SPEED_FF_K            1.2f
#define YAW_SPEED_FF_MAX          300.0f
#define YAW_CURRENT_FF_K          1.0f
#define YAW_CURRENT_FF_MAX        1500.0f

#define YAW_TARGET_SPEED_LPF_TAU  0.02f
#define YAW_TARGET_ACC_LPF_TAU    0.03f

#define PITCH_GRAVITY_FF_K        1.0f
#define PITCH_GRAVITY_FF_FUCTION_A 0.0f
#define PITCH_GRAVITY_FF_FUCTION_B 0.0f

typedef struct
{
    float dt;
    float last_target_yaw;

    float target_yaw_speed;
    float target_yaw_speed_lpf;

    float last_target_yaw_speed_lpf;
    float target_yaw_acc;
    float target_yaw_acc_lpf;

    uint32_t last_tick;
    uint8_t initialized;
    uint8_t valid;
} YawFeedforwardState_t;

typedef struct
{
    float yaw_speed_ff;
    float yaw_current_ff;
    float pitch_current_ff;
} Gimbal_Feedforward_s;

void YawFeedforwardUpdate(float target_yaw);
float PitchGravityFeedforwardUpdate(const float actual_pitch);

#endif
