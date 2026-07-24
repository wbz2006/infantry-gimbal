#ifndef GIMBAL_H
#define GIMBAL_H

#include "gimbal_algorithm.h"

extern Gimbal_Feedforward_s gimbal_ff;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H