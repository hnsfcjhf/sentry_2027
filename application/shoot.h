/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2023     YXC              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "referee.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "detect_task.h"
//=================PID============================
// 拨弹轮电机PID
#define TRIGGER_MOTOR_PID_KP            1500.0f
#define TRIGGER_MOTOR_PID_KI            0.0f
#define TRIGGER_MOTOR_PID_KD            0.0f
// 射击上弹PID输出限幅
#define TRIGGER_READY_PID_MAX_OUT       10000.0f
#define TRIGGER_READY_PID_MAX_IOUT      7000.0f
// 射击PID输出限幅
#define TRIGGER_BULLET_PID_MAX_OUT      10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT     9000.0f

// 射击摩擦轮PID
#define SHOOT_PID_KP                    7800.0f//8500.0f//7500.0f
#define SHOOT_PID_KI                    2.5f//10.0f
#define SHOOT_PID_KD                    200.0f//3000.0f
#define SHOOT_PID_MAX_OUT               16000.0f
#define SHOOT_PID_MAX_IOUT              1450.0f
//=================END============================

//丢失目标时间
#define Shoot_Vision_Change_Time_Sett  100


// 空闲一段时间
#define SHOOT_TASK_INIT_TIME    100
// shoot task control time  10ms
// 射击任务控制间隔 10ms
#define SHOOT_CONTROL_TIME_MS   1
//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL   1
//射击摩擦轮打开 关闭
#define SHOOT_ON_KEYBOARD       KEY_PRESSED_OFFSET_Z
#define SHOOT_OFF_KEYBOARD      KEY_PRESSED_OFFSET_X
// 射速上限
#define SHOOT_SPEED_LIMIT           25.0f//30.0f
// 一次射击热量增加值
#define SHOOT_ONE_HEAT              10.0f
// 持续射击热量每次运行增加值
#define SHOOT_CUNTINUE_HEAT         70.0f
//鼠标长按判断
#define PRESS_LONG_TIME             200
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例(TODO)
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f

#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18

//摩擦轮电机转速一致系数
#define FRIC_SAME_L_UP                 0.0041288f//0.0041263f//0.0041293f//左摩擦轮        
#define FRIC_SAME_L_DOWN               0.0041287f//0.0039883f//0.0041287f//0.0039223f//0.004223f//0.0040723f//0.0041287f       
#define FRIC_SAME_R_UP                 0.0041400f//右摩擦轮       
#define FRIC_SAME_R_DOWN               0.0041265f        


//拨弹速度
#define TRIGGER_SPEED               3.0f               // 拨一颗子弹转速
#define CONTINUE_TRIGGER_SPEED      10.0f//7.5f//-8.0f//-12.0f // 持续射击转速
#define READY_TRIGGER_SPEED         3.0f               // 子弹上膛转速
// 摩擦轮转速增加值
#define SHOOT_FRIC_RAMP_ADD_VALUE   15.0f
// 微动开关开、关
#define SWITCH_TRIGGER_ON           1
#define SWITCH_TRIGGER_OFF          0

#define BLOCK_TRIGGER_SPEED         1.0f        // 卡弹转速
#define BLOCK_TIME                  150         // 卡弹时间
#define REVERSE_TIME                80          // 反转时间

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.698


#define REVIVAL_TIME 7000

#define speed_baohu 1

#define Trigger_Lengque             30//枪管冷却时间

typedef enum
{
    SHOOT_STOP = 0,             // 射击停止
    SHOOT_READY_FRIC,           // 摩擦轮准备
    SHOOT_READY_BULLET,         // 子弹上膛
    SHOOT_READY,                // 射击准备
    SHOOT_BULLET,               // 射击一发子弹
    SHOOT_CONTINUE_BULLET,      // 持续射击
    SHOOT_AUTO,                  // 自动射击
	
	  SHOOT_auto_B,
	
    SHOOT_DONE,                 // 射击结束
} shoot_mode_e; // 射击模式

typedef enum
{
  SHOOT_Q_STOP = 0,             //停止状态
	SHOOT_Q_READY_DETECT,         //准备检测状态
	SHOOT_Q_SUSPECT,              //发射嫌疑状态
	SHOOT_Q_CONFIRM,              //确认发射状态
} shoot_q_status_e; // 热量检测模式

typedef enum
{
		NO_TURNBACK=0,							//没有枪管卡弹
    TRIGGER_TURNBACK1,					//枪管1卡弹
		TRIGGER_TURNBACK2,					//枪管2卡弹
		OVERHEAT2_TURNBACK1,				//枪管2超热量且枪管1卡弹
		OVERHEAT1_TURNBACK2,				//枪管1超热量且枪管2卡弹
} shoot_turnback_mode; // 射击模式

typedef struct
{
    const motor_measure_t *shoot_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} shoot_motor_t;    // 摩擦轮电机数据

typedef struct
{
    const motor_measure_t* trigger_motor_measure;    // 拨弹电机数据指针
    fp32 speed;                                      // 拨弹轮转速
		fp32 last_speed_set;
    fp32 speed_set;                                  // 拨弹轮转速设置值
    int16_t give_current;                            // 拨弹电机给的电流值
    fp32 angle;
    fp32 angle_set;
}trigger_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;                        // 射击模式
		shoot_mode_e last_shoot_mode;
	  shoot_q_status_e shoot_q1_status;                // 热量检测模式
	  shoot_q_status_e shoot_q2_status;
    const RC_ctrl_t *shoot_rc;                      // 遥控器指针
    uint8_t last_s;                                 // 遥控器上一次拨的档位
    shoot_motor_t shoot_motor[4];                   // 摩擦轮数据
    trigger_motor_t trigger_motor;                  // 拨弹电机数据
    trigger_motor_t trigger2_motor;                  // 拨弹电机数据

    pid_type_def trigger_motor_pid;                 // 拨弹轮PID
    pid_type_def trigger2_motor_pid;                 // 拨弹轮PID

    pid_type_def fric_motor_pid[4];                 // 摩擦轮PID

    ramp_function_source_t fric1_ramp;              // 摩擦轮1斜坡函数
    ramp_function_source_t fric2_ramp;              // 摩擦轮2斜坡函数
	  ramp_function_source_t fric3_ramp;              // 摩擦轮3斜坡函数
    ramp_function_source_t fric4_ramp;              // 摩擦轮4斜坡函数
	
    fp32 fric_speed1_set;
    fp32 fric_speed2_set;
	fp32 fric_speed3_set;
    fp32 fric_speed4_set;

    fp32 fric_speed_rate;                           // 射速比例
    
    // 俩摩擦轮电机转速一致系数
    fp32 fric_same_R;
    fp32 fric_same_L;
    
    bool_t key;                                     // 微动开关状态

    uint16_t heat_limit;                            // 当前热量限制
    uint16_t heat;                                  // 当前热量
		uint16_t last_heat;                                  
		uint16_t heat_change;
		

	uint16_t heat_add;
	uint16_t last_heat_add;
//    int32_t trigger_given_current;
//    int32_t fric1_given_current;
//    int32_t fric2_given_current;

    fp32 F_speed;                                   // 两摩擦轮平均速度

    int8_t ecd_count;

    bool_t mouse_l;                 // 鼠标左键按下状态
    bool_t mouse_r;                 // 鼠标右键按下状态
    bool_t last_mouse_l;            // 上一次鼠标左键按下状态
    bool_t last_mouse_r;            // 上一次鼠标右键按下状态
    uint16_t mouse_l_time;          // 鼠标左键按下时间
    uint16_t mouse_r_time;          // 鼠标右键按下时间
    uint16_t rc_s_time;             // 遥控器拨下档时间
    
    uint16_t block_time;            // 卡弹时间
    uint16_t block_time2;            // 卡弹时间

    uint16_t reverse_time;          // 反转时间
    uint16_t reverse_time2;          // 反转时间

    bool_t shoot_turn_flag;         // 一颗子弹射出标志
		uint8_t turn_back_flag;
		
    float shoot_17_speed;        // 当前射出的子弹的速度
    float shoot_17_speed_last;   // 上一颗射出的子弹的速度
		
}shoot_control_t;

extern shoot_control_t shoot_control;
/**
  * @brief          shoot task, osDelay SHOOT_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          射击任务，间隔 SHOOT_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void shoot_task(void const *pvParameters);

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      shoot_control_init:"shoot_control_t"变量指针.
 * @retval         返回空
 */
void shoot_init(shoot_control_t *shoot_control_init);

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      shoot_control_update:"shoot_control_t"变量指针.
 * @retval         返回空
 */
void shoot_feedback_update(shoot_control_t *shoot_control_update);

/**
 * @brief          射击模式设置
                    遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗.
                    一直处在下，则持续发射，用于3min准备时间清理子弹.
 * @param[in]      shoot_control_mode:"shoot_control_t"变量指针.
 * @retval         void
 */
void shoot_set_mode(shoot_control_t *shoot_control_mode);

/**
 * @brief          设置射击控制量
 * @param[in]      set_control:"shoot_control_t"变量指针.
 * @retval         void
 */
void shoot_set_control(shoot_control_t *set_control);

/**
 * @brief          计算射击数据
 * @param[in]      control_loop:"shoot_control_t"变量指针.
 * @retval         void
 */
void shoot_control_loop(shoot_control_t *control_loop);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
void shoot_bullet_control(void);

/**
  * @brief          自动射击控制
  * @param[in]      void
  * @retval         void
  */

/**
  * @brief          堵转处理函数
  * @param[in]      void
  * @retval         void
  */
void trigger_motor_turn_back(void);
void trigger2_motor_turn_back(void);

#endif
