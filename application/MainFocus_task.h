#ifndef __MAINFOCUS_H
#define __MAINFOCUS_H
#include "stm32f4xx.h"
#include "INS_task.h"
#define SEND_RATE   100000

typedef struct
{
    int32_t euler_angle[3];         // 欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
    int32_t angle_speed[3];         // 角速度, 0:x轴, 1:y轴, 2:roll轴 单位 rad/s
    int32_t accel_speed[3];         // 加速度, 0:x轴, 1:y轴, 2:roll轴 单位 m/s2
    uint8_t camp;                   // 机器人阵营，'r'——红方，'b'——蓝方
    uint8_t shoot_mode;             // 射击模式，'a'代表打装甲板，f代表打符（默认'a'）
    uint8_t last_shoot_mode;        // 上一次射击模式
    uint8_t send_data[49];    		// 发送缓存区
	

	
}auto_send_t;  					 	// 自瞄发送数据

typedef struct
{
    volatile uint8_t receive_end_flag;  // 一帧数据接收完成标志
//    uint8_t ReceiveData_bit[24];        // 接收原始二进制数据缓存区
//    int32_t ReceiveData[2][3];         // 接收数据解算缓存区(原始)，[0]——yaw，[1]——pitch
//    
//    fp32 absolute_angle_auto_P;     // 自瞄时的绝对角度值（rad）
//    fp32 auto_angle_P;              // 自瞄角度（0——360°）
//    fp32 auto_angle_rad_P;          // 自瞄角度（弧度制角度0——2PI（6.28））
//    
//    fp32 absolute_angle_auto_Y;     // 自瞄时的绝对角度值（rad）
//    fp32 auto_angle_Y;              // 自瞄角度（0——360°）
//    fp32 auto_angle_rad_Y;          // 自瞄角度（弧度制角度0——2PI（6.28））
	
	fp32 auto_x;
	fp32 auto_y;
	fp32 auto_PP;
	fp32 auto_YY;
	
	uint8_t vision_flag;
	uint8_t last_vision_flag;
	uint8_t shoot_auto;
	uint8_t aim_flag;
	
	uint8_t Auto_Cap_U;
	uint8_t auto_position;
	uint8_t auto_revolve;
	uint8_t	auto_last_revolve;
	uint8_t auto_revolve_back;
	uint8_t auto_upslope;
	uint8_t auto_downslope;
	
}auto_receive_t;    // 自瞄接收数据
    

typedef struct
{
    auto_send_t auto_send;
//    auto_receive_t auto_receive;

}auto_manage_t;

extern auto_send_t auto_send;
extern uint8_t Lock_flag,shoot_stop_flag,lose_flag;//锁定标志位  丢失标志位
extern uint32_t lose_count;
extern  float  PP;
extern  float  YY;

extern fp32 vision_p;
extern fp32 vision_y;



extern fp32 last_pitch_ab_angle;//todo
extern fp32 last_yaw_ab_angle;


extern fp32 auto_chassis_x,auto_chassis_y,auto_chassis_wz,auto_vx_init;//wgz
extern int8_t test_aim;
extern auto_receive_t  auto_receive;
extern auto_manage_t auto_manage;

extern uint8_t sentry_mode_set;
//extern auto_manage_t auto_manage;

void MainFocus_task(void const * argument );

//typedef struct
//{
//    int8_t  PitchAxiaAngle_H8;
//	uint8_t  PitchAxiaAngle_L8;
//    int8_t  YawAxiaAngle_H8;
//	uint8_t  YawAxiaAngle_L8;
//    int8_t  Mode_H8;
//	int8_t  Mode_L8;
//	float YawAxiaAngle;
//	float PitchAxiaAngle;
//	int16_t  Mode;
//} Auto_data ;

#endif
