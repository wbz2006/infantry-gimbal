# gimbal



## 工作流程

初始化pitch和yaw电机以及一个imu。订阅gimbal_cmd消息（来自robot_cmd）并发布gimbal_feed话题。

1. 从消息中心获取gimbal_cmd话题的消息
2. 根据消息中的控制模式进行模式切换，如果急停则关闭所有电机
3. 由设定的模式，进行电机反馈数据来源的切换，修改反馈数据指针，设置前馈控制数据指针等。
4. 设置反馈数据，包括yaw电机的绝对角度和imu数据
5. 推送反馈数据到gimbal_feed话题下
## 故障反馈

App_Fault_s 使用 daemon 中的公共定义，云台电机位 1 为 DJI Yaw，位 2 为 DM Pitch，其余位为 0；在线查询统一使用 MotorIsOffline，分别传入各自的反馈 daemon 和 CAN 实例。通信字段报告 50 ms 内没有有效 CMD 命令或命令内容无效，IMU 字段来自 INS_GetFaultFlags（采样超过 10 ms 未更新或数据无效）。不用的控制输入字段为 0。任一故障使云台零力矩，并清除 PID 历史；故障反馈仍持续发布。feedback_valid 表示电机和 IMU 反馈可用，命令链路故障单独由 fault 表达。

GimbalInit 只通过 INS_GetData 获取姿态地址。SPI 初始化与校准由 INS 任务在中断开启后执行，初始化失败也能继续运行通信并反馈 IMU 故障，不再卡住整板启动。
