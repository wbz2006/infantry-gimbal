#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include "daemon.h"

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

/**
 * @brief 读取 CMD 自身的反馈通信、遥控器等分类故障，具体执行机构故障见各应用反馈。
 * @return const App_Fault_s* 由 CMD 每周期更新的故障状态地址。
 */
const App_Fault_s *RobotCMDGetFault(void);

#endif // !ROBOT_CMD_H
