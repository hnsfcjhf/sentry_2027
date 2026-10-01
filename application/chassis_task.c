/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.1.0     Nov-22-2023     YXC             1. change mode and delete "chassis_behaviour.c"
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "referee.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "gimbal_behaviour.h"
#include "MainFocus_task.h"
#include "chassis_power_control_with_supercap.h"
#include "CAN_supercap_communication.h"
#include "kalman.h"
#include "referee.h"
#include "usart.h"
#include "stdio.h"


uint8_t uart_temp[7] ={"1:0\n"} ;

#define rc_deadband_limit(input, output, dealine)       \
    {                                                       \
        if ((input) > (dealine) || (input) < -(dealine))    \
        {                                                   \
            (output) = (input);                             \
        }                                                   \
        else                                                \
        {                                                   \
            (output) = 0;                                   \
        }                                                   \
    }

		/**
 * @brief 计算两个角度之间的最小误差（弧度），结果在 [-PI, PI] 范围内
 * @param target 目标角度 (rad)
 * @param current 当前角度 (rad)
 * @return 规范化的角度误差 (target - current)
 */
static inline fp32 angle_error_normalized(fp32 target, fp32 current) {
    fp32 error = target - current;
    // 将误差规范到 [-PI, PI]
    while (error > PI) error -= 2.0f * PI;
    while (error < -PI) error += 2.0f * PI;
    return error;
}
		
/**
 * @brief          底盘初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);
/**
 * @brief          底盘模式设置
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
 * @brief          底盘数据更新
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
 * @brief          底盘控制量设置
 * @param[out]     chassis_move_control:"chassis_move"变量指针.
 * @retval         none
 */
 
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
 * @brief          底盘控制PID计算，并发送电流值
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
//static void chassis_control_loop_with_SuperCap(chassis_move_t *chassis_move_control_loop);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
/**
 * @brief          底盘功率更新
 * @param[out]     chassis_move_init_t:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_power_update(chassis_move_t *chassis_move_init_t);
/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */
static void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

// 针对fp32（float）类型的符号函数实现
//static int sgn(fp32 x) ;

void DebugOut_UART_chassis(void);

#if J_scope_TEST_MODE
    // j-scope 帮助pid调参
//    static void J_scope_test(void);
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t chassis_high_water;    //
#endif

// 底盘移动模式
//chassis_move_mode_e Chassis_move_mode;
uint8_t Chassis_move_mode;

// 底盘运动数据
chassis_move_t chassis_move;

UART_chassis_TxDataTypeDef sUART_TxData_chassis;

CAN_SuperCapTXDataTypeDef CAN_SuperCap_Send;

// 小陀螺标志
uint8_t revolve_flag = 0,revolve_direction_flag=0;
uint8_t No_Move_flag = 0;
// 小陀螺回正标志
uint8_t revolve_back_flag = 0;
void chassis_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 底盘初始化
    chassis_init(&chassis_move);

    // 判断底盘电机是否都在线，如果有一个不在线，就停那
    // 比赛时不用，因为宁愿只用其他三个轮子也不让它停在那不动
//    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
//    {
//             vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }

    while (1)
    {
        // 更新底盘功率
        chassis_power_update(&chassis_move);
        // 设置底盘控制模式
        chassis_set_mode(&chassis_move);
        // 模式切换数据保存，以保证模式切换平稳过渡
        chassis_mode_change_control_transit(&chassis_move);
        // 底盘数据（底盘速度、加速度、姿态角）更新
        chassis_feedback_update(&chassis_move);
        // 底盘控制量设置，根据键盘或遥控器计算出不同模式各个电机的速度
        chassis_set_contorl(&chassis_move);
			
				chassis_control_loop(&chassis_move);


        // make sure one motor is online at least, so that the control CAN message can be received
        // 确保至少一个电机在线， 这样CAN控制包可以被接收到
        // if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        // {
        // when remote control is offline, chassis motor should receive zero current.
        // 当遥控器掉线的时候，发送给底盘电机零电流.
//        if (toe_is_error(DBUS_TOE))
//        {
//            CAN_cmd_chassis(0, 0, 0, 0);
////            CAN_cmd_chassis(600, 600, 600, 600);
//        }
        if (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE))
				CAN_cmd_chassis(0, 0, 0, 0);
        else
        {
            // 发送控制电流
  //         CAN_cmd_chassis(0, 0, 0, 0);
          CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                            chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        }

        //}
				
//				DebugOut_UART_chassis();   //vofa+调试

        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
        //osDelay(CHASSIS_CONTROL_TIME_MS);

        #if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}

float revolve_value;
float MAX_WHEEL_SPEED;
float go_straight;
float NORMAL_MAX_CHASSIS_SPEED_X; // 底盘前进运动速度上限
float NORMAL_MAX_CHASSIS_SPEED_Y; // 底盘平移运动速度上限
float REVOLVE_MAX_CHASSIS_SPEED_X;
float REVOLVE_MAX_CHASSIS_SPEED_Y;

// 超级电容通信所需要的
int robort_HP = 1;    // 血量标志 0——无血， 1——有血
int text_T = 0;     // 测试数据变量
float chassis_power_NEW;
float chassis_power_buffer_NEW;
extern uint8_t Power_flag;  // 超级电容反馈是否可以开启
int open = 0;   // 是否已开启超级电容标志（0——未开启，1——已开启）



// 底盘初始化
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    // 底盘速度环pid值
   const static fp32 motor_speed_pid[4][3] = {
    {M3505_MOTOR1_SPEED_PID_KP, M3505_MOTOR1_SPEED_PID_KI, M3505_MOTOR1_SPEED_PID_KD},
    {M3505_MOTOR2_SPEED_PID_KP, M3505_MOTOR2_SPEED_PID_KI, M3505_MOTOR2_SPEED_PID_KD},
    {M3505_MOTOR3_SPEED_PID_KP, M3505_MOTOR3_SPEED_PID_KI, M3505_MOTOR3_SPEED_PID_KD},
    {M3505_MOTOR4_SPEED_PID_KP, M3505_MOTOR4_SPEED_PID_KI, M3505_MOTOR4_SPEED_PID_KD}
	  };
    // 底盘功率控制PID
    const static fp32 motor_speed_POWER_pid[3] = {SPEED_POWER_PID_KP, SPEED_POWER_PID_KI, SPEED_POWER_PID_KD};
    // 底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 wheel_same[4] = {WHEEL_SAME_1, WHEEL_SAME_2, WHEEL_SAME_3, WHEEL_SAME_4};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    // 底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_ZERO_FORCE;

    // 获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    // 获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    // 获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

    // 获取底盘电机数据指针，初始化PID
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        // 获取底盘电机数据指针
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        // PID初始化
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid[i], M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    
        chassis_move_init->chassis_wheel_same[i] = wheel_same[i];
    }

    // 初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    // 初始化功率限制PID
    PID_init(&chassis_move_init->motor_speed_POWER_pid, PID_POSITION, motor_speed_POWER_pid, SPEED_POWER_PID_MAX_OUT, SPEED_POWER_PID_MAX_IOUT);

    // 用一阶滤波代替斜波函数生成
    //** 防止速度增大过快（幅度太大，易损坏电机，且可能超功率）让速度缓慢增加（键盘遥控才需要）
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		
    // 最大 最小速度
    chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    // 更新一下数据
    chassis_feedback_update(chassis_move_init);
		
//		KalmanCreate(&chassis_move_init->wz_filter, 0.01f, 0.1f); 
}


/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;

    for (i = 0; i < 4; i++)
    {
        // 更新电机速度，加速度是速度的PID微分		速度越大*更大的比例，速度增大，加强pid的计算减小的更多
        //** CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    底盘电机转速转换为轮子转速系数
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_move_update->chassis_wheel_same[i];
//        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    // 更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[3].speed - chassis_move_update->motor_chassis[2].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

//		float wz_raw = (-chassis_move_update->motor_chassis[0].speed 
//               - chassis_move_update->motor_chassis[1].speed 
//               - chassis_move_update->motor_chassis[2].speed 
//               - chassis_move_update->motor_chassis[3].speed) 
//               * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 
//               / MOTOR_DISTANCE_TO_CENTER;

// 应用卡尔曼滤波，得到平滑后的 wz
//chassis_move_update->wz = KalmanFilter(&chassis_move_update->wz_filter, wz_raw);
		
    // 计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    //** 底盘上没有陀螺仪，所以其姿态角由云台姿态角度及底盘电机编码值解算得到
    //** chassis_yaw底盘yaw轴角度      =        陀螺仪角度        -       云台电机编码器的值
    chassis_move_update->chassis_yaw   = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET)   - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll  = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}


/**
 * @brief          底盘功率更新
 * @param[out]     chassis_move_init_t:"chassis_move"变量指针.
 * @retval         none
 */
uint16_t chassis_power_limit_tT = 100,use_power_limit=0;
float k_t_t;
uint8_t move_flag=0;

static void chassis_power_update(chassis_move_t *chassis_move_init_t)
{
		No_Move_flag=0;
    // 获取裁判系统返回的功率限制
    get_chassis_power_limit(&chassis_power_limit_tT);//如果数据变了，超功率，就在这里减20
	  
//	  chassis_power_limit_tT=150;//手动赋值裁判功率限制
		revolve_value=1.3f;
    //-----------------------------血量-移动判断--------
    //--------------------------超级电容通信--------------------------
		
	if(chassis_move_init_t->chassis_mode==CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW||chassis_move_init_t->chassis_mode==CHASSIS_NO_FOLLOW_YAW)//跟随云台模式
	{
		if((chassis_move_init_t->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] > 40||chassis_move_init_t->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]<-40)||
			(chassis_move_init_t->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] > 40||chassis_move_init_t->chassis_RC->rc.ch[CHASSIS_X_CHANNEL]<-40)
			||(gimbal_control.gimbal_rc_ctrl->rc.ch[YAW_CHANNEL]>25||gimbal_control.gimbal_rc_ctrl->rc.ch[YAW_CHANNEL]<-25)
		)
		{
			move_flag=1;//正在移动
		}
		else
		{
			move_flag=0;
		}
					
		if(revolve_flag==1)//开了小陀螺
		{
			use_power_limit=chassis_power_limit_tT;//+100;//提高功率
		//	use_power_limit=50;
			if(move_flag)    //同时在移动
			{
				revolve_value=1.3f;//降低转速
			}
		}
		else               //没开小陀螺
		{
			if(move_flag)
				use_power_limit=chassis_power_limit_tT;//正常功率限制
			else
				use_power_limit=0;//0;//0使用功率
		}
//		if (switch_is_up(chassis_move_init_t->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])&&move_flag!=0)
//		use_power_limit += 100;
//		use_power_limit=chassis_power_limit_tT;
	}
	else if(chassis_move_init_t->chassis_mode==Sentry_Auto_Move_Run){//自动模式时
					//走得慢 充电
					if(auto_receive.Auto_Cap_U==1)//根据视觉信息控制
					{
							use_power_limit= chassis_power_limit_tT/3;//chassis_power_limit_tT-100;//低功率
					}
					//不使用超电
					else if(auto_receive.Auto_Cap_U==2)
					{
							use_power_limit=chassis_power_limit_tT;//正常功率
					}
					//超电放电
					else if(auto_receive.Auto_Cap_U==3)
					{
							use_power_limit=chassis_power_limit_tT+100;//chassis_power_limit_tT;//超功率
					}	
					else//超电以裁判系统全部功率作为充电功率的情况下
					{
						use_power_limit=chassis_power_limit_tT;//-30
					}
					//偷存能量
					if(get_supercap_energy()<=45)//超电能量较低
					{
						if(get_robot_HP()>145)//且血量较高时
						{
							if(auto_receive.Auto_Cap_U==3)//且需要超功率时
							use_power_limit=chassis_power_limit_tT;//使用正常功率
						}
						else//且血量较低时
						{
							use_power_limit=use_power_limit;//相当于不改变功率上限（正常执行视觉的信息控制功率150W）
						}
					}
//					use_power_limit=chassis_power_limit_tT;
					revolve_value=1.3f;
	}

		set_chassis_power(use_power_limit);//底盘超电功率控制
	
		MAX_WHEEL_SPEED=5.0f;//<2.85f

		NORMAL_MAX_CHASSIS_SPEED_X = MAX_WHEEL_SPEED;
    NORMAL_MAX_CHASSIS_SPEED_Y = MAX_WHEEL_SPEED;	
		
				
	chassis_move_init_t->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init_t->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	
	chassis_move_init_t->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init_t->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
		
		CAN_SuperCap_Send.Enable=ENABLE;//ENABLE
		CAN_SuperCap_Send.Powerlimit=chassis_power_limit_tT;
	  CAN_SuperCap_Send.ChargePower=chassis_power_limit_tT;
	
		CAN_cmd_SuperCap(&CAN_SuperCap_Send);
}

/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
chassis_mode_e last_chassis_mode;

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    
		 last_chassis_mode=chassis_move_mode->chassis_mode;
		
	if(toe_is_error(DBUS_TOE))//遥控器离线
	{
		if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL])||get_game_progress()==4||switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			chassis_move_mode->chassis_mode = Sentry_Auto_Move_Run;//哨兵自动模式
		}
	}
	else if(!toe_is_error(DBUS_TOE))
	{
		 // 遥控器设置模式
		if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;   // 底盘无力
			revolve_flag=0;
		}
		else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			chassis_move_mode->chassis_mode = CHASSIS_NO_FOLLOW_YAW;      // 底盘跟随云台   CHASSIS_NO_FOLLOW_YAW
		}
		else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
		{
			chassis_move_mode->chassis_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;   // 底盘跟随云台
		}
	}
	

    // when gimbal in some mode, such as init mode, chassis must's move
    // 当云台在某些模式下，像初始化、云台校准、云台静止、云台无力时， 底盘不动
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_FORCE;   // CHASSIS_NO_MOVE;
    }
}


/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    // 如果上一个模式等于要设置的模式就直接返回(跳出)
    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    // 切入底盘跟随云台模式
    // 上一个模式不等于要设置的模式且要设置的模式等于对应的模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        // 设置云台和底盘之间的相对角度为0
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
	else if(chassis_move_transit->chassis_mode ==Sentry_Auto_Move_Run)
	{
		 chassis_move_transit->chassis_relative_angle_set = 0.0f;
	}
    // 切入云台不跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    // 切入底盘不移动模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_NO_MOVE) && chassis_move_transit->chassis_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        // 云台跟随底盘角度模式 和 不跟随云台模式的底盘yaw角度都由底盘控制，故设置一样
    }
		
    // 更新一下底盘数据，上一个模式 == 现在模式
    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


    fp32 vx_set_channel, vy_set_channel;
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    int16_t vx_channel, vy_channel;
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    //    CHASSIS_RC_DEADLINE     摇杆死区(10)
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    // CHASSIS_VX_RC_SEN    遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例 (0.006f)
    vx_set_channel = -vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = -vy_channel *CHASSIS_VY_RC_SEN;//

//    vx_set_channel = vx_channel /660 * MAX_WHEEL_SPEED;
//    vy_set_channel = -vy_channel * CHASSIS_VY_RC_SEN;

    // 键盘控制
//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY) // 前
//    {
//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
//    }
//    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY) // 后
//    {
//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
//    }

//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY) // 左
//    {
//        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
//    }
//    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY) // 右
//    {
//        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
//    }

    // first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    // stop command, need not slow change, set zero derectly
    // 停止信号，不需要缓慢加速，直接减速到零
//        if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
//        {  
//                chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
//        }
//        if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
//        {
//                chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
//        }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}


/**
 * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
 * @param[out]     chassis_move_update:"chassis_move"变量指针.
 * @retval         none
 */
float vx_set_t=0;
float vy_set_t=0;
fp32 angle_set_init;
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    // get three control set-point, 获取三个控制设置值		angle_set用于小陀螺时随时加的	
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

	vx_set_t=vx_set;
	vy_set_t=vy_set;
		if ((last_chassis_mode != CHASSIS_NO_FOLLOW_YAW && chassis_move_control->chassis_mode == CHASSIS_NO_FOLLOW_YAW) || (last_chassis_mode != Sentry_Auto_Move_Run && chassis_move_control->chassis_mode == Sentry_Auto_Move_Run))
		angle_set_init =-gimbal_control.gimbal_yaw_motor.absolute_angle;
    // 跟随云台模式
//    if (switch_is_up(chassis_move_control->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    if (chassis_move_control->chassis_mode == CHASSIS_NO_FOLLOW_YAW || chassis_move_control->chassis_mode == Sentry_Auto_Move_Run)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        
        if (revolve_flag == 0 && auto_receive.auto_revolve==0)    // 未开启小陀螺时
        {
            // rotate chassis direction, make sure vertial direction follow gimbal
            // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
						//这里保证的跟随分解云台相对坐标系成为底盘相对坐标
					//所以不论底盘是否与云台的方向一致，只要给了相对角度和地盘坐标系下的x和y速度就能往想要的方向走。
//					//故需要将底盘坐标系转换成云台坐标系，确保跟随云台
            sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
            cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
            chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
            chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
//					    chassis_move_control->vx_set = vx_set;
//					    chassis_move_control->vy_set = vy_set;
								//防止大幅转向回来时超功率的问题
//								if(revolve_back_flag)
//								{//每次调整0.1f角度
//									if(chassis_move_control->chassis_yaw_motor->relative_angle <3.14f &&chassis_move_control->chassis_yaw_motor->relative_angle>0)
//										angle_set = chassis_move_control->chassis_yaw_motor->relative_angle + rad_format(angle_set);
//									else if(chassis_move_control->chassis_yaw_motor->relative_angle > -3.14f&&chassis_move_control->chassis_yaw_motor->relative_angle<0)
//										angle_set = chassis_move_control->chassis_yaw_motor->relative_angle - rad_format(angle_set);
//									if(fabs(chassis_move_control->chassis_yaw_motor->relative_angle)<=0.1f)
//									{//直到yaw电机相对角度小于0.1f
//										revolve_back_flag=0;//清除标志位
//										angle_set=0.0f;//转速清零
//									}
//								}
//								else
//								{
//									angle_set=0.0f;
//								}
								
//								if(chassis_move_control->chassis_yaw_motor->relative_angle>=0.035 || chassis_move_control->chassis_yaw_motor->relative_angle<=-0.035)   //设置死区，减小底盘跟随云台的微小震荡
//								{
// 计算期望的相对角度（基于云台绝对角度和附加偏置）
chassis_move_control->chassis_relative_angle_set = rad_format(angle_set_init) + gimbal_control.gimbal_yaw_motor.absolute_angle;

// 计算规范化角度误差
fp32 angle_error = angle_error_normalized(chassis_move_control->chassis_relative_angle_set, 
                                          chassis_move_control->chassis_yaw_motor->relative_angle);

// 将目标角度调整为与当前角度在同一周期，以避免 PID 输入跳变
fp32 adjusted_target = chassis_move_control->chassis_yaw_motor->relative_angle + angle_error;

				if(auto_receive.auto_upslope==1)
				{
					chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
				}
        else {
// 计算角速度指令
chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 
                                        chassis_move_control->chassis_yaw_motor->relative_angle, 
                                        adjusted_target);}//								}
//								else chassis_move_control->wz_set = 0;
        }
        // 小陀螺用
        else 
        {
					
            sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
            cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);

            chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
            chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

            // 设置控制底盘旋转的角度
            chassis_move_control->chassis_relative_angle_set = rad_format(angle_set) + chassis_move_control->chassis_yaw_motor->relative_angle;
            // 计算旋转PID角速度
            chassis_move_control->wz_set = 3 * (revolve_value * (PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set)));

            //	chassis_move_control->wz_set = chassis_power_limit_buffer_pid_proportion*5*(revolve_value*(-PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set)));
            //  //第二种方法，直接给一个确定的遥控值,直接给的是速度
            //	chassis_move_control->wz_set = CHASSIS_WZ_RC_SEN * angle_set;
        }

        // speed limit
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		
		   else if (chassis_move_control->chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        
        if (revolve_flag == 0)    // 未开启小陀螺时
        {
            // rotate chassis direction, make sure vertial direction follow gimbal
            // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
						//这里保证的跟随分解云台相对坐标系成为底盘相对坐标
					//所以不论底盘是否与云台的方向一致，只要给了相对角度和地盘坐标系下的x和y速度就能往想要的方向走。
//					//故需要将底盘坐标系转换成云台坐标系，确保跟随云台
            sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
            cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
            chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
            chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
//					    chassis_move_control->vx_set = vx_set;
//					    chassis_move_control->vy_set = vy_set;
								//防止大幅转向回来时超功率的问题
//								if(revolve_back_flag)
//								{//每次调整0.1f角度
//									if(chassis_move_control->chassis_yaw_motor->relative_angle <3.14f &&chassis_move_control->chassis_yaw_motor->relative_angle>0)
//										angle_set = chassis_move_control->chassis_yaw_motor->relative_angle + rad_format(angle_set);
//									else if(chassis_move_control->chassis_yaw_motor->relative_angle > -3.14f&&chassis_move_control->chassis_yaw_motor->relative_angle<0)
//										angle_set = chassis_move_control->chassis_yaw_motor->relative_angle - rad_format(angle_set);
//									if(fabs(chassis_move_control->chassis_yaw_motor->relative_angle)<=0.1f)
//									{//直到yaw电机相对角度小于0.1f
//										revolve_back_flag=0;//清除标志位
//										angle_set=0.0f;//转速清零
//									}
//								}
//								else
//								{
									angle_set=0.0f;
//								}
								
								chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
								chassis_move_control->wz_set =  PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);

//								else chassis_move_control->wz_set = 0;
        }
        // 小陀螺用
        else if (revolve_flag == 1)
        {
					
            sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
            cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);

            chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
            chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

            // 设置控制底盘旋转的角度
            chassis_move_control->chassis_relative_angle_set = rad_format(angle_set) + chassis_move_control->chassis_yaw_motor->relative_angle;
            // 计算旋转PID角速度
            chassis_move_control->wz_set = 3 * (revolve_value * (PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set)));

            //	chassis_move_control->wz_set = chassis_power_limit_buffer_pid_proportion*5*(revolve_value*(-PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set)));
            //  //第二种方法，直接给一个确定的遥控值,直接给的是速度
            //	chassis_move_control->wz_set = CHASSIS_WZ_RC_SEN * angle_set;
        }

        // speed limit
        // 速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		
//	else if(chassis_move_control->chassis_mode == Sentry_Auto_Move_Run)
//	{
//		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
//			
//			// 小陀螺用
//			if (auto_receive.auto_revolve == 1||revolve_flag==1)
//			{
//				
//				sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
//				cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);

//				chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
//				chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

//				// 设置控制底盘旋转的角度
//				chassis_move_control->chassis_relative_angle_set = rad_format(angle_set) + chassis_move_control->chassis_yaw_motor->relative_angle;

//								//  //第二种方法，直接给一个确定的遥控值,直接给的是速度
//					chassis_move_control->wz_set = 3 * (revolve_value * (PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set)));

//			}
//			else if (auto_receive.auto_revolve == 0)    // 未开启小陀螺时
//			{
//				// rotate chassis direction, make sure vertial direction follow gimbal
//				// 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
//				sin_yaw = arm_sin_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
//				cos_yaw = arm_cos_f32(chassis_move_control->chassis_yaw_motor->relative_angle);
//				chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
//				chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;

//				// set control relative angle  set-point
//				// 设置控制相对云台角度
//				
//					chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);

//				if(auto_receive.auto_upslope==1)
//				{
//					chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
//				}
//				else
//				{
//					chassis_move_control->wz_set = 0;//angle_set;
//				}
//			}
//			
//			

//			// speed limit
//			// 速度限幅
//			chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//			chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
//		
//        
//	}
    // 底盘不动
    else if (chassis_move_control->chassis_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_control->vx_set = vx_set;      // 0.0f
        chassis_move_control->vy_set = vy_set;      // 0.0f
        chassis_move_control->wz_set = angle_set;   // 0.0f
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        // in raw mode, set-point is sent to CAN bus
        // 在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;      // 0.0f
        chassis_move_control->vy_set = vy_set;      // 0.0f
        chassis_move_control->wz_set = angle_set;   // 0.0f
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
 * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
 * @param[out]     vx_set, 通常控制纵向移动.
 * @param[out]     vy_set, 通常控制横向移动.
 * @param[out]     wz_set, 通常控制旋转运动.
 * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
 * @retval         none
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    // 底盘无力, 跟没上电那样
    if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *angle_set = 0.0f;
    }
    // 底盘不动
    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_MOVE)
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *angle_set = 0.0f;
    }
    // 底盘跟随云台转动
    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW || chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
    {
		if(last_chassis_mode==Sentry_Auto_Move_Run )//上次是自动模式
		{
			revolve_flag=0;
		}

        // channel value and keyboard value change to speed set-point, in general
        // 遥控器的通道值以及键盘按键 得出 一般情况下的！速度设定值！
        chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
        // 360旋转(小陀螺)
        static fp32 revolve_add_angle = 0.0f;//转速

        // 按下鼠标按键ctrl或者拨动拨动轮大于500
        if ((chassis_move_rc_to_vector->chassis_RC->key.v & REVOLVE_START_KEY) || chassis_move_rc_to_vector->chassis_RC->rc.ch[4] > 500)
        {
					if(revolve_direction_flag==0)//正转					1
					{
						revolve_add_angle=0.12f;//用于控制旋转方向与速度
					}
					else if(revolve_direction_flag==1)//反转		4
					{
						revolve_add_angle=-0.12f;
					}
					revolve_flag=1;//开启小陀螺
					
        }
        else if ((chassis_move_rc_to_vector->chassis_RC->key.v & REVOLVE_STOP_KEY) || chassis_move_rc_to_vector->chassis_RC->rc.ch[4] < -500)
        {//拨动  拨动轮  小于-500
						if(revolve_direction_flag==0)//上一次是正转				
						{                            //就以反转方向回归云台方向
							revolve_add_angle=-0.12f;			//2 -0.075f
							if(revolve_flag==1)
							{
							revolve_direction_flag=1;
							}
						}
						else													//上一次是反转
						{
							revolve_add_angle=-0.12f;		//3  -0.075f
							if(revolve_flag==1)
							{
							revolve_direction_flag=0;
							}
						}
						revolve_flag=0;//关闭小陀螺
						revolve_back_flag=1;//底盘回正至云台方向
        }
		
        *angle_set = revolve_add_angle;
    }
	else if(chassis_move_rc_to_vector->chassis_mode == Sentry_Auto_Move_Run)
	{
		#if TEST_MODE
		
			if(auto_receive.auto_revolve==1)//串口数据开启小陀螺
			{
				*angle_set= 0.1f;
			}			
			
		 if(auto_receive.auto_upslope==1||auto_receive.auto_downslope==1)//爬坡时反转小陀螺
		{
				*angle_set= - 0.1f;
		}
		//一阶低通滤波，对用遥控控的进行清零处理。
//		if(chassis_move_rc_to_vector->chassis_mode != Sentry_Auto_Move_Run)
//		{
//			chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out=0;
//			chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out=0; 
//		}
//		
//		 first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, auto_chassis_x);
//		 first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, auto_chassis_y);

//			*vx_set=chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
//			*vy_set=chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
				*vx_set=auto_chassis_x;
		
  			*vy_set=auto_chassis_y;
		#else
			if(get_game_progress()==4 || switch_is_mid(chassis_move_rc_to_vector->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
			{
//				if (auto_receive.aim_flag > 0 && get_robot_HP() >= 150)//发现目标
//				{
//					auto_chassis_x=0;//底盘不动
//					auto_chassis_y=0;
//					revolve_flag=1;//开启小陀螺
//				}
//				else
//				{
//				 revolve_flag=0;//不转
//				}
				
				if(auto_receive.auto_revolve==1||revolve_flag==1)
				{
					*angle_set= 0.12f;
				}
				else
				{
					*angle_set= 0.0f;
				}
		//一阶低通滤波，对用遥控控的进行清零处理。
		if(chassis_move_rc_to_vector->chassis_mode != Sentry_Auto_Move_Run)
		{
			chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out=0;
			chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out=0;
		}
		
		 first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, auto_chassis_x);
		 first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, auto_chassis_y);
    if((auto_chassis_x!=0||auto_chassis_y!=0)&&sentry_mode_set!=0)
		revolve_value=0.7f;
		
		//			*vx_set=chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
//			*vy_set=chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
		
			*vx_set=-auto_chassis_x;
			*vy_set=auto_chassis_y;
//				
			} 
//			else
//			{
//				chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
//				*angle_set=0;
			}
		#endif
		
	//}
//    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
//    {
//        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//    }
    // 底盘不跟随角度，底盘旋转速度由参数直接设定
    else if (chassis_move_rc_to_vector->chassis_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
        *angle_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
    }
    else
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *angle_set = 0.0f;
    }
}


/**
 * @brief          控制循环，根据超电功率反馈计算出来的最大轮限制速控制设定值，计算电机电流值，进行控制，只有在超电在线的时候才会运行
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */

// static void chassis_control_loop_with_SuperCap(chassis_move_t *chassis_move_control_loop)
// {
//     fp32 max_vector = 0.0f, vector_rate = 0.0f;
//     fp32 temp = 0.0f;
//     fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//     uint8_t i = 0;
//     // 车量运动分解到各个轮子速度
//     chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

//     // 底盘无力
//     if (chassis_move_control_loop->chassis_mode == CHASSIS_ZERO_FORCE)
//     {
//         for (i = 0; i < 4; i++)
//         {
//             chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
//         }
//         // raw控制直接返回
//         return;
//     }
//     // 计算轮子控制最大速度，并限制其最大速度
//     for (i = 0; i < 4; i++)     //** 找出最大速度
//     {
//         chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
//         temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);

// 			//得到四个轮子里最大的一个速度
//         if (max_vector < temp)
//         {
//             max_vector = temp;
//         }

//     }

// 		/*通过max_vector（四个轮子中最大的一个）去判断它是否超过限制，超过了，
// 		用最大的得到一个最小的值，只要轮子最大速度的一个被限制住了，其他的轮子速度也会被限制住*/
//     // 速度限幅，避免超过最大速度
//     if (max_vector > MAX_WHEEL_SPEED)
//     {
//         vector_rate = MAX_WHEEL_SPEED / max_vector;

//         for (i = 0; i < 4; i++)
//         {
//             chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
//         }
//     }
//     // 计算pid
//     for (i = 0; i < 4; i++)
//     {
//         PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
//     }
	
//   for (i = 0; i < 4; i++)
//     {
// 		//motor_chassis[i].give_current用于pid计算
//         chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
//     }
// }


/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
int test=0;
extern power_heat_data_t power_heat_data;
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //float chassis_power_buffer_NEW;
    // mecanum wheel speed calculation
    // 车量运动分解到各个轮子速度
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set, chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    // 底盘无力
    if (chassis_move_control_loop->chassis_mode == CHASSIS_ZERO_FORCE)
    {
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }

        // in raw mode, derectly return
        // raw控制直接返回
        return;
    }

    // calculate the max speed in four wheels, limit the max speed
    // 计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)     //** 找出最大速度
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);

        if (max_vector < temp)
        {
            max_vector = temp;
        }

    }

    // 速度限幅，避免超过最大速度
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;

        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    // calculate pid
    // 计算pid
    for (i = 0; i < 4; i++)
    {
//        chassis_move_control_loop->motor_chassis[i].speed_set =2;
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
   }
		
    // 功率控制，获取功率和缓冲能量
    get_chassis_power_and_buffer(&chassis_power_NEW, &chassis_power_buffer_NEW);
//		k_t_t=chassis_move_control_loop->motor_speed_POWER_pid.out;
		
		k_t_t = 	(chassis_power_buffer_NEW-20)/60.0f;//缓冲能量比例

    if (k_t_t <= 0)
        k_t_t = 0; 
	
	test++;
//功率控制
//	for (i = 0; i < 4; i++)
//	{
//		chassis_power_control(chassis_move_control_loop);
//	}
//	power_heat_data.chassis_power;
//	
chassis_power_control_with_supercap(chassis_move_control_loop);	//超电功率控制
 #if chassis_baohu

   for (i = 0; i < 4; i++)
     {
 		//motor_speed_pid[i].out实际发送出去的can*k_t_t
 		//motor_chassis[i].give_current用于pid计算
         chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
     }
 #else

     // 赋值电流值
     for (i = 0; i < 4; i++)
     {
         chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
     }
 #endif
}

/**
 * @brief          四个麦轮速度是通过三个参数计算出来的
 * @param[in]      vx_set: 纵向速度
 * @param[in]      vy_set: 横向速度
 * @param[in]      wz_set: 旋转速度
 * @param[out]     wheel_speed: 四个麦轮速度
 * @retval         none
 */


static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{ 
    // because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    // 旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快

    // 全向轮
//    wheel_speed[0] = (vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[1] = (-vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[2] = (vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[3] = (-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
	
//	  wheel_speed[0] =   -vx_set - vy_set + wz_set * (CHASSIS_WZ_SET_SCALE - 1.0f)*MOTOR_DISTANCE_TO_CENTER;//右上逆时针
//    wheel_speed[1] =    vx_set - vy_set + wz_set * (CHASSIS_WZ_SET_SCALE - 1.0f)*MOTOR_DISTANCE_TO_CENTER;//前0.8
//    wheel_speed[2] =    vx_set + vy_set + wz_set * (CHASSIS_WZ_SET_SCALE - 1.0f)*MOTOR_DISTANCE_TO_CENTER;//右0.8
//    wheel_speed[3] =   -vx_set + vy_set + wz_set * (CHASSIS_WZ_SET_SCALE - 1.0f)*MOTOR_DISTANCE_TO_CENTER;//后
    // 麦克纳母轮
    wheel_speed[0] =   -vx_set - vy_set + (wz_set) * (car_length + car_width) * MOTOR_DISTANCE_TO_CENTER; //左
    wheel_speed[1] =    vx_set - vy_set + (wz_set) * (car_length + car_width) * MOTOR_DISTANCE_TO_CENTER; //前
    wheel_speed[2] =    vx_set + vy_set + (wz_set) * (car_length + car_width) * MOTOR_DISTANCE_TO_CENTER; //右
    wheel_speed[3] =   -vx_set + vy_set + (wz_set) * (car_length + car_width) * MOTOR_DISTANCE_TO_CENTER; //后  
}

float last_in = 0;
float T = 0.002;
float J = 0.4332;
float forwardfeed(float in,float j)
{
  float out;
  out = j *(in - last_in)/T + in;
  last_in = in;
  return out;
}



#if J_scope_TEST_MODE
int32_t chassis_motor_speed_1000[4], chassis_motor_speed_set_1000[4];
int32_t chassis_angle_1000, chassis_angle_set_1000;

int32_t yaw_absolute_angle_1000;
int32_t yaw_absolute_angle_set_1000;
int32_t yaw_motor_gyro_1000;
int32_t yaw_motor_gyro_set_1000;

int32_t chassis_relative_angle_set_1000;
int32_t chassis_relative_angle_1000;

//static void J_scope_test(void)
//{
//    uint16_t i;



////    chassis_angle_1000 = (int32_t)(chassis_move.chassis_yaw * 1000);
////    chassis_angle_set_1000 = (int32_t)(chassis_move.chassis_yaw_set * 1000);
//	
//	
//	yaw_absolute_angle_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle *1000);
//	yaw_absolute_angle_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set *1000);

//	yaw_motor_gyro_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro *1000);
//	yaw_motor_gyro_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set *1000);
//	
//	chassis_relative_angle_set_1000 = (int32_t)(chassis_move.chassis_relative_angle_set *1000);
//	chassis_relative_angle_1000 = (int32_t)(chassis_move.chassis_yaw_motor->relative_angle *1000);
//	
////	for (i = 0; i < 4; i++)
////    {
////        chassis_motor_speed_1000[i] = (int32_t)(chassis_move.motor_chassis[i].speed * 1000);
////        chassis_motor_speed_set_1000[i] = (int32_t)(chassis_move.motor_chassis[i].speed_set * 1000);
////    }

//	
//}
#endif



// 针对fp32（float）类型的符号函数实现
//static int sgn(fp32 x) {
//    if (x > 1e-6) {  // 用极小值1e-6避免浮点精度误差（如x=0.0000001误判为0）
//        return 1;
//    } else if (x < -1e-6) {
//        return -1;
//    } else {
//        return 0;  // x接近0时返回0
//    }
//}

  void DebugOut_UART_chassis(void) {
    sUART_TxData_chassis.ch_data[0] = (float)chassis_move.motor_chassis[0].speed;
    sUART_TxData_chassis.ch_data[1] = (float)chassis_move.motor_chassis[1].speed;
    sUART_TxData_chassis.ch_data[2] = (float)chassis_move.motor_chassis[2].speed;
    sUART_TxData_chassis.ch_data[3] = (float)chassis_move.motor_chassis[3].speed;
    sUART_TxData_chassis.ch_data[4] = (float)chassis_move.motor_chassis[0].speed_set;
    sUART_TxData_chassis.ch_data[5] = (float)chassis_move.motor_chassis[1].speed_set;
    sUART_TxData_chassis.ch_data[6] = (float)chassis_move.motor_chassis[2].speed_set;
    sUART_TxData_chassis.ch_data[7] = (float)chassis_move.motor_chassis[3].speed_set;

    sUART_TxData_chassis.ch_data[8] = (float)chassis_move.motor_chassis[0].give_current;//CAN_SuperCapRXData.ChassisPower;
    sUART_TxData_chassis.ch_data[9] = (float)chassis_move.motor_chassis[1].give_current;//CAN_SuperCapRXData.SuperCapEnergy;
    sUART_TxData_chassis.ch_data[10] = (float)chassis_move.motor_chassis[2].give_current;//InitialTotalPower;
    sUART_TxData_chassis.ch_data[11] = (float)chassis_move.motor_chassis[3].give_current;

    sUART_TxData_chassis.tail[0] = 0x00;
    sUART_TxData_chassis.tail[1] = 0x00;
    sUART_TxData_chassis.tail[2] = 0x80;
    sUART_TxData_chassis.tail[3] = 0x7f;

    HAL_UART_Transmit(&huart1, (uint8_t *)&sUART_TxData_chassis, sizeof(sUART_TxData_chassis), sizeof(sUART_TxData_chassis));
}
