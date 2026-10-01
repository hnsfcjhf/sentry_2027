/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "kalman.h"

#define Sup_Test 0
//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
#define CHASSIS_CONTROL_TIME 0.002f
//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//in some mode, can use remote control to control rotation speed
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2
//the channel of choosing chassis mode,
//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL 0

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f//0.006f.
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.006f//0.006f//0.002f
//in following yaw angle mode, rocker value add to angle 
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000005f
//in not following yaw angle mode, rocker value change to rotation speed
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.02f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//摇杆死区1
#define CHASSIS_RC_DEADLINE 40

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
// 轮子距中心的距离（TODO）
#define MOTOR_DISTANCE_TO_CENTER 0.24996f//0.22981f//0.2f

//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508 rmp change to chassis speed,
//m3508电机转化成底盘轮子速度(m/s)的比例（TODO）
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f//0.000567691304069734//0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

#define CHASSIS_WZ_SET_SCALE 0.1f

//chassis forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY   KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY    KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY    KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY   KEY_PRESSED_OFFSET_D
//底盘360旋转按键
#define REVOLVE_START_KEY   KEY_PRESSED_OFFSET_CTRL
#define REVOLVE_STOP_KEY    KEY_PRESSED_OFFSET_E

//chassis motor speed PID
//底盘电机速度环PID
#define M3505_MOTOR1_SPEED_PID_KP 9000.0f//7500.0f//7280.0f//9000.0f//8000.0f
#define M3505_MOTOR1_SPEED_PID_KI 0.0f//21.0f//40.0f//5.0f
#define M3505_MOTOR1_SPEED_PID_KD 0.0f

#define M3505_MOTOR2_SPEED_PID_KP 9000.0f//7500.0f//7720.0f//9000.0f//8000.0f
#define M3505_MOTOR2_SPEED_PID_KI 0.0f//21.0f//40.0f//5.0f
#define M3505_MOTOR2_SPEED_PID_KD 0.0f

#define M3505_MOTOR3_SPEED_PID_KP 9000.0f//7500.0f//7280.0f//9000.0f//8000.0f
#define M3505_MOTOR3_SPEED_PID_KI 0.0f//21.0f//40.0f//5.0f
#define M3505_MOTOR3_SPEED_PID_KD 0.0f

#define M3505_MOTOR4_SPEED_PID_KP 9000.0f//7500.0f//7720.0f//9000.0f//8000.0f
#define M3505_MOTOR4_SPEED_PID_KI 0.0f//21.0f//40.0f//5.0f
#define M3505_MOTOR4_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 35.0f//15.2f//15.2f//13.2f//原来25.0f//15.0f//10.0f//10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f//0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 5.0f//0.0f//250.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 50.0f//22.0f//50.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f//0.0f

// 功率限制PID
/*Iout和out的限幅不能太大，因为如果你实际超过了功率，
但是没限制住就会一下超以下底盘超功率断电，然后又没有对out和iout输出清零，导致他上线是直接给个最大的轮速，又超。*/
#define SPEED_POWER_PID_KP  0.0001f//
#define SPEED_POWER_PID_KI 0.0003f//0.0001f//
#define SPEED_POWER_PID_KD 0.0f//
#define SPEED_POWER_PID_MAX_OUT 5.0f		//5.0f
#define SPEED_POWER_PID_MAX_IOUT 5.0f	//5.0f

// 四个轮子转速一致系数（按ID顺序） 系数越小，速度越小
#define WHEEL_SAME_1    1.0f//0.65f//0.97//1.01f
#define WHEEL_SAME_2    1.0f//0.75f//0.97f//1.0f
#define WHEEL_SAME_3    1.0f//1.05f//0.99f
#define WHEEL_SAME_4   1.0f//0.60f//1.0f//0.962f

#define car_length 0.1775//0.2
#define car_width 0.176//0.15

#define chassis_baohu 1
typedef enum
{
    // 底盘无力, 跟没上电那样.chassis will be like no power.
    CHASSIS_ZERO_FORCE,
    // 底盘保持不动.chassis will be stop.
    CHASSIS_NO_MOVE,
    // 底盘跟随云台.chassis will follow gimbal.
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
    // 工程底盘角度控制底盘，由于底盘未有陀螺仪，故而角度是减去云台角度而得到，
    // 如果有底盘陀螺仪请更新底盘的yaw，pitch，roll角度 在chassis_feedback_update函数中
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,    // （未用）
    // 底盘不跟随角度，角度是开环的，但轮子是有速度环.
    CHASSIS_NO_FOLLOW_YAW,   

	Sentry_Auto_Move_Run//自动模式
	
} chassis_mode_e;

typedef enum
{
    // 快速移动，功率超过裁判系统功率限制，超出部分由超级电容模块补充
    RUN =0,
	
    // 正常移动，功率等于裁判系统功率限制
    NORMAL,
	
    // 慢速移动，功率小于裁判系统功率限制，分配一部分功率给超级电容充电
    SLOW,
		
		// 不移动
    STOP    // （未用）

} chassis_move_mode_e;




typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
  chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据
  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid
  pid_type_def	motor_speed_POWER_pid;

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  extKalman_t wz_filter;
	
  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
  fp32 chassis_relative_angle_set;  //the set relative angle.设置相对云台控制角度
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度
  
  fp32 chassis_wheel_same[4];   // 四个轮子转速一致系数

} chassis_move_t;

typedef struct {
    float ch_data[12];
    uint8_t tail[4];
} UART_chassis_TxDataTypeDef;

extern UART_chassis_TxDataTypeDef sUART_TxData_chassis;
extern chassis_move_t chassis_move;
extern uint8_t revolve_flag;
/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);





#endif
