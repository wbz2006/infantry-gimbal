#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "can_comm.h"
#include "gimbal_algorithm.h"
#include "serial_print.h"

/*云台外设定义*/
static attitude_t *gimbal_IMU_data;               // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;
/*消息订阅与发布定义*/
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static Subscriber_t *chassis_imu_sub;       // 底盘IMU消息订阅者
static attitude_t chassis_imu_recv;         // 接收的底盘IMU数据
/*云台控制算法定义*/
// Yaw轴速度前馈 + Yaw轴加速度前馈 + Pitch轴重力补偿前馈
Gimbal_Feedforward_s gimbal_ff;



// static BMI088Instance *bmi088; // 云台IMU

void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 4,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 65, // 8
                .Ki = 0,
                .Kd = 1,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            .speed_feedforward_ptr = &gimbal_ff.yaw_speed_ff,
            .current_feedforward_ptr = &gimbal_ff.yaw_current_ff,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_AND_SPEED_FEEDFORWARD,
        },
        .motor_type = GM6020
    };
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x06,
            .rx_id = 0x302,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.5,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 10,
            },
            .speed_PID = {
                .Kp = 0.3,  
                .Ki = 0, 
                .Kd = 0,   
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0,
                .MaxOut = 1.5,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[0]),
            .current_feedforward_ptr = &gimbal_ff.pitch_current_ff,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = DM4310,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    pitch_motor->mit_config.velocity_des = 0.0f;
    pitch_motor->mit_config.position_des = 0.0f;
    pitch_motor->mit_config.kd           = 0.0f;
    pitch_motor->mit_config.kp           = 0.0f;
    pitch_motor->mit_config.torque_des   = 0.0f;

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    chassis_imu_sub = SubRegister("chassis_imu", sizeof(attitude_t));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(chassis_imu_sub, &chassis_imu_recv);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DMMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        // DJIMotorEnable(yaw_motor);
        // DMMotorEnable(pitch_motor);
        // DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        // DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        // DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        // DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        // //使用底盘imu进行前馈，云台imu数据进行反馈，对底盘的扰动进行补偿
        // GimbalStabilityCalc(gimbal_IMU_data, &chassis_imu_recv, &gimbal_comp);

        // DJIMotorSetRef(yaw_motor, gimbal_comp.yaw_speed); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DMMotorSetRef(pitch_motor, gimbal_comp.pitch_speed);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DMMotorEnable(pitch_motor);

        // YawSpeedFeedforwardUpdate(gimbal_cmd_recv.yaw);
        PitchGravityFeedforwardUpdate(gimbal_IMU_data->Pitch);

        YawFeedforwardUpdate(gimbal_cmd_recv.yaw);

        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DMMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        
        float error = gimbal_cmd_recv.yaw - gimbal_IMU_data->YawTotalAngle;
        SerialPrintf("%.1f,%.1f,%.1f,%.1f,%.1f\n",gimbal_IMU_data->Yaw,  gimbal_cmd_recv.yaw, 
                                     gimbal_IMU_data->Pitch, gimbal_cmd_recv.pitch, error);
        break;
    default:
        break;
    }
    

    
    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
