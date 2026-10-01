/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2023     YXC              1.separate shot task and gimbal task
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee.h"

#include "shoot.h"
#include "gimbal_behaviour.h"
#include "MainFocus_task.h"

// 微动开关IO   1——按下（有子弹）  0——松开（无子弹）
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

shoot_control_t shoot_control;
uint8_t melee_state = 0;
int melee_state_add = 0;
fp32 FRIC_speed = 0;   // 射速限制

uint8_t other_robot_live=0;

//#if J_scope_TEST_MODE
//    // j-scope 帮助pid调参
//    static void J_scope_test(void);
//#endif

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
void shoot_task(void const *pvParameters)
{
    // 空闲一段时间
    vTaskDelay(SHOOT_TASK_INIT_TIME);
    // 射击初始化
    shoot_init(&shoot_control);

    while (1)
    {
        // 射击模式设置
        shoot_set_mode(&shoot_control);
        // 数据更新
        shoot_feedback_update(&shoot_control);
        // 设置射击控制量
        shoot_set_control(&shoot_control);
        // 计算射击数据
        shoot_control_loop(&shoot_control);

//        if (toe_is_error(DBUS_TOE))
//        {
//            CAN_cmd_shoot(0, 0, 0, 0);
////            CAN_cmd_shoot(1600, 1600, 1600, 1600);
//        }
//        else
//        {
//            CAN_cmd_shoot(0, 0, 0, 0);
//            CAN_cmd_shoot(600, 600, 600, 600);
//            CAN_cmd_shoot(-shoot_control.shoot_motor[0].give_current, shoot_control.shoot_motor[1].give_current, shoot_control.trigger_motor.give_current, 0);
			  CAN_cmd_fric(shoot_control.shoot_motor[0].give_current,-shoot_control.shoot_motor[1].give_current,-shoot_control.shoot_motor[2].give_current, shoot_control.shoot_motor[3].give_current);
			//-shoot_control.shoot_motor[1].give_current,,,-shoot_control.shoot_motor[1].give_current,
//
        CAN_cmd_trigger( 0,0,shoot_control.trigger_motor.give_current ,shoot_control.trigger2_motor.give_current );//
//				CAN_cmd_fric(0,0,0,0);
			//-shoot_control.shoot_motor[1].give_current,,,-shoot_control.shoot_motor[1].give_current,
//			  CAN_cmd_trigger( 0,0,0 ,0 );//
//        }
//					CAN_cmd_trigger( 0,0,600,0);



        // 系统延时
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
    }
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      init:"shoot_control_t"变量指针.
 * @retval         返回空
 */
void shoot_init(shoot_control_t *init)
{
    uint8_t i;
    // 拨弹轮PID
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_MOTOR_PID_KP, TRIGGER_MOTOR_PID_KI, TRIGGER_MOTOR_PID_KD};

    // 摩擦轮PID
    static const fp32 shoot_speed_pid[3]   = {SHOOT_PID_KP, SHOOT_PID_KI, SHOOT_PID_KD};
    // 初始化射击模式——SHOOT_STOP
    init->shoot_mode = SHOOT_STOP;
    // 获取遥控器指针
    init->shoot_rc = get_remote_control_point();
    // 获取拨弹轮电机指针
    init->trigger_motor.trigger_motor_measure = get_trigger_motor_measure_point();
	  init->trigger2_motor.trigger_motor_measure = get_trigger2_motor_measure_point();

    // 初始化拨弹轮PID
    PID_init(&init->trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&init->trigger2_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);

    // 获取摩擦轮电机指针、初始化PID
    for (i = 0; i < 4; i++)
    {
        init->shoot_motor[i].shoot_motor_measure = get_shoot_motor_measure_point(i);
        PID_init(&init->fric_motor_pid[i], PID_POSITION, shoot_speed_pid, SHOOT_PID_MAX_OUT, SHOOT_PID_MAX_IOUT);

        init->shoot_motor[i].speed_set = init->shoot_motor[i].speed = 0;
        init->shoot_motor[i].give_current = 0;
    }

//    init->fric_same_R = FRIC_SAME_R;    // 左右速度比
//    init->fric_same_L = FRIC_SAME_L;

    init->fric_speed_rate = 1;  // 速度比例默认100%
    init->shoot_17_speed = 0;           // 射速
    init->shoot_17_speed_last = 0;      // 上一次的射速

    // 摩擦轮斜坡函数初始化
    ramp_init(&init->fric1_ramp, SHOOT_CONTROL_TIME_MS * 0.001f, FRIC_speed, 0);
    ramp_init(&init->fric2_ramp, SHOOT_CONTROL_TIME_MS * 0.001f, FRIC_speed, 0);
		ramp_init(&init->fric3_ramp, SHOOT_CONTROL_TIME_MS * 0.001f, FRIC_speed, 0);
    ramp_init(&init->fric4_ramp, SHOOT_CONTROL_TIME_MS * 0.001f, FRIC_speed, 0);
	
    init->trigger_motor.angle_set = init->trigger_motor.angle = init->trigger_motor.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;;
    init->trigger_motor.speed = init->trigger_motor.speed_set = 0.0f;
	  init->trigger2_motor.angle_set = init->trigger2_motor.angle = init->trigger2_motor.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;;
    init->trigger2_motor.speed = init->trigger2_motor.speed_set = 0.0f;

    init->ecd_count = 0;
    init->shoot_turn_flag = 0;  // 一颗子弹射出标志

    init->trigger_motor.give_current = 0;
	  init->trigger_motor.give_current = 0;


    init->key = BUTTEN_TRIG_PIN;
    
    // 更新数据
    shoot_feedback_update(init);
}

/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      shoot_update:"shoot_control_t"变量指针.
 * @retval         返回空
 */
char speed_flag=0;
char speed_down=0;
void shoot_feedback_update(shoot_control_t *shoot_update)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
	
	  static fp32 speed_fliter_11 = 0.0f;
    static fp32 speed_fliter_22 = 0.0f;
    static fp32 speed_fliter_33 = 0.0f;

    // 拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    // 二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_update->trigger_motor.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_update->trigger_motor.speed = speed_fliter_3; // 拨弹轮转速

		speed_fliter_11 = speed_fliter_22;
    speed_fliter_22 = speed_fliter_33;
    speed_fliter_33 = speed_fliter_22 * fliter_num[0] + speed_fliter_11 * fliter_num[1] + (shoot_update->trigger2_motor.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_update->trigger2_motor.speed =speed_fliter_33; // 拨弹轮转速

    // 控制摩擦轮速度
    shoot_update->shoot_motor[0].speed =  shoot_update->shoot_motor[0].shoot_motor_measure->speed_rpm * FRIC_SAME_L_DOWN;
    shoot_update->shoot_motor[2].speed = -shoot_update->shoot_motor[2].shoot_motor_measure->speed_rpm * FRIC_SAME_L_UP;
		shoot_update->shoot_motor[1].speed = -shoot_update->shoot_motor[1].shoot_motor_measure->speed_rpm * FRIC_SAME_R_DOWN;
    shoot_update->shoot_motor[3].speed =  shoot_update->shoot_motor[3].shoot_motor_measure->speed_rpm * FRIC_SAME_R_UP;

    // 获取枪口当前热量限制和热量
		get_shoot_heat_limit(&shoot_update->heat_limit);
    get_shooter_17mm_power_heat_data(&shoot_update->heat);

    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_update->trigger_motor.trigger_motor_measure->ecd - shoot_update->trigger_motor.trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_update->ecd_count--;
    }
    else if (shoot_update->trigger_motor.trigger_motor_measure->ecd - shoot_update->trigger_motor.trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_update->ecd_count++;
    }

    if (shoot_update->ecd_count == FULL_COUNT)
    {
        shoot_update->ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_update->ecd_count == -FULL_COUNT)
    {
        shoot_update->ecd_count = FULL_COUNT - 1;
    }

    // 计算输出轴角度
    shoot_update->trigger_motor.angle = (shoot_update->ecd_count * ECD_RANGE + shoot_update->trigger_motor.trigger_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    shoot_update->trigger2_motor.angle =(shoot_update->ecd_count * ECD_RANGE + shoot_update->trigger2_motor.trigger_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    // 微动开关IO口电平更新
    shoot_update->key = BUTTEN_TRIG_PIN;

    // 鼠标按键更新
    shoot_update->last_mouse_l = shoot_update->mouse_l;
    shoot_update->last_mouse_r = shoot_update->mouse_r;
    shoot_update->mouse_l = shoot_update->shoot_rc->mouse.press_l;
    shoot_update->mouse_r = shoot_update->shoot_rc->mouse.press_r;
	
	
// 获取这一颗子弹射速
    Get_shooter_17mm_speed(&shoot_update->shoot_17_speed);
//    switch ((int)shoot_update->shoot_17_speed) //      射速（2022.7.13）shoot_update->short_17_speed
	switch (25)
    {
    case 0:
        FRIC_speed = 0;
        break;
    case 15:
        FRIC_speed = 19.5;
        break; // 10.6f
    case 18:
        FRIC_speed = 20.8f;
        break;
    case 22:
        FRIC_speed = 24.4f;
        break;
    case 25:
        FRIC_speed = 24.4f;//29.4f;//27.4f;//24.4f
        break; // 29.4
    case 90:
        FRIC_speed = 19.0f;
        break;
    default:
        FRIC_speed = 24.4f;
        break;
    }
	
//-----------------------------------
#if speed_baohu
//	       if (shoot_update->shoot_17_speed >= SHOOT_SPEED_LIMIT&&speed_flag==0)
//   {
//		speed_down++;
//	   speed_flag=1;  
//		   }
//   
//		   if(shoot_update->shoot_17_speed != shoot_update->shoot_17_speed_last)speed_flag=0;
//    shoot_update->shoot_17_speed_last = shoot_update->shoot_17_speed;
//	shoot_update->fric1_ramp.max_value = FRIC_speed-speed_down;
//    shoot_update->fric2_ramp.max_value = FRIC_speed-speed_down;
//	shoot_update->fric3_ramp.max_value = FRIC_speed-speed_down;
//    shoot_update->fric4_ramp.max_value = FRIC_speed-speed_down;
	
	shoot_update->fric1_ramp.max_value = FRIC_speed;
    shoot_update->fric2_ramp.max_value = FRIC_speed;
	shoot_update->fric3_ramp.max_value = FRIC_speed;
    shoot_update->fric4_ramp.max_value = FRIC_speed;

#else
    shoot_update->fric1_ramp.max_value = FRIC_speed;
    shoot_update->fric2_ramp.max_value = FRIC_speed;
	shoot_update->fric3_ramp.max_value = FRIC_speed;
    shoot_update->fric4_ramp.max_value = FRIC_speed;
		   //-----------------------
    
#endif
    //-------------------------------长按计时判断--------------------------
    //鼠标长按计时
    if (shoot_update->mouse_l)
    {
        if (shoot_update->mouse_l_time < PRESS_LONG_TIME)
            shoot_update->mouse_l_time++;
    }
    else
        shoot_update->mouse_l_time = 0;

    //遥控器拨下档计时
    if (switch_is_down(shoot_update->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if (shoot_update->rc_s_time < PRESS_LONG_TIME)
            shoot_update->rc_s_time ++;
    }
    else
        shoot_update->rc_s_time = 0;
}


/**
 * @brief          射击模式设置
                    遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗.
                    一直处在下，则持续发射，用于3min准备时间清理子弹.
 * @param[in]      shoot_control_mode:"shoot_control_mode_t"变量指针.
 * @retval         void
 */
uint16_t press_l_shoot_flag = 0;
uint8_t need_shoot_flag;
int Shoot_Auto_Change_Time=0;//Shoot_Vision_Change_Time_Sett+1;
extern robot_status_t robot_status;
extern game_robot_HP_t game_robot_HP;

fp32 ttii=0;
uint16_t shoot_time_delay=0;

short blue_1_1hp=10;
short blue_1_2hp=10;
int blue_1_time=0;
char blue_1_flag=0;


short blue_3_1hp=10;
short blue_3_2hp=10;
int blue_3_time=0;
char blue_3_flag=0;


short blue_4_1hp=10;
short blue_4_2hp=10;
int blue_4_time=0;
char blue_4_flag=0;


short red_1_1hp=10;
short red_1_2hp=10;
int red_1_time=0;
char red_1_flag=0;


short red_3_1hp=10;
short red_3_2hp=10;
int red_3_time=0;
char red_3_flag=0;


short red_4_1hp=10;
short red_4_2hp=10;
int red_4_time=0;
char red_4_flag=0;

//uint8_t t,tt=0;
//uint32_t h,hh;

void shoot_set_mode(shoot_control_t *shoot_control_mode)
{
	if (switch_is_down(shoot_control_mode->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
	{
		shoot_control_mode->shoot_mode = SHOOT_STOP;
	}
    else if (switch_is_mid(shoot_control_mode->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control_mode->shoot_mode = SHOOT_READY_FRIC;//此时摩擦轮以开始转动
    }
    else if (switch_is_up(shoot_control_mode->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control_mode->shoot_mode = SHOOT_READY_BULLET;//此时拨弹轮开始转动
    }
    //判断两个摩擦轮斜波函数输出转速是否达标
    if (	  shoot_control_mode->shoot_mode == SHOOT_READY_BULLET
            && shoot_control_mode->fric1_ramp.out == shoot_control_mode->fric1_ramp.max_value
            && shoot_control_mode->fric2_ramp.out == shoot_control_mode->fric2_ramp.max_value
						&& shoot_control_mode->fric3_ramp.out == shoot_control_mode->fric3_ramp.max_value
            && shoot_control_mode->fric4_ramp.out == shoot_control_mode->fric4_ramp.max_value)
	{
			shoot_control_mode->shoot_mode = SHOOT_CONTINUE_BULLET;
//		  if (t == 0)
//			{				h = HAL_GetTick();
//				      t = 1;
//			}
//			if (shoot_control_mode->shoot_motor[0].speed<23.85f&&tt==0)
//			{tt=1;
//				hh = HAL_GetTick();
//			}
	}
	if(toe_is_error(DBUS_TOE)	)
	{
//		#if TEST_MODE
			shoot_control.shoot_mode = SHOOT_READY_FRIC;
			if(auto_receive.aim_flag == 1)//瞄到了//(fabs(vision_p)<5 && fabs(vision_y)<5 &&
			{
				if(auto_receive.vision_flag==3)//(auto_receive.vision_flag==3)//陀螺仪当前角度与视觉发送角度（绝对角度）差值
				{
					need_shoot_flag = 1;
				}
				else
				{  
					need_shoot_flag = 0;
				}
	
			}
			else 	need_shoot_flag = 0;
			
//		//存活状态
//if(get_robot_term()==1)//红方
//	{		//英雄工程步兵哨兵
//		if(game_robot_HP.blue_1_robot_HP>0) other_robot_live = other_robot_live|0x01;
//			else other_robot_live=other_robot_live & (~0x01);
//		if(game_robot_HP.blue_2_robot_HP>0) other_robot_live = other_robot_live|(0x01<<1);//写入
//			else other_robot_live=other_robot_live & (~0x02);
//		if(game_robot_HP.blue_3_robot_HP>0) other_robot_live = other_robot_live|(0x01<<2);
//			else other_robot_live=other_robot_live & (~0x04);
//		if(game_robot_HP.blue_4_robot_HP>0) other_robot_live = other_robot_live|(0x01<<3);
//			else other_robot_live=other_robot_live & (~0x08);
//		if(game_robot_HP.blue_5_robot_HP>0) other_robot_live = other_robot_live|(0x01<<4);
//			else other_robot_live=other_robot_live & (~0x10);
//		if(game_robot_HP.blue_outpost_HP>0) other_robot_live = other_robot_live|(0x01<<5);
//			else other_robot_live=other_robot_live & (~0x20);
//		if(game_robot_HP.blue_7_robot_HP>0) other_robot_live = other_robot_live|(0x01<<6);
//			else other_robot_live=other_robot_live         & (~0x40);
//		if(game_robot_HP.blue_base_HP>0) other_robot_live = other_robot_live|(0x01<<7);
//			else other_robot_live=other_robot_live & (~0x80);
//	}
//	else if(get_robot_term()==2)//蓝方
//	{
//		if(game_robot_HP.red_1_robot_HP>0) other_robot_live = other_robot_live|0x01;
//			else other_robot_live=other_robot_live & (~0x01);
//		if(game_robot_HP.red_2_robot_HP>0) other_robot_live = other_robot_live|(0x01<<1);//写入
//			else other_robot_live=other_robot_live & (~0x02);
//		if(game_robot_HP.red_3_robot_HP>0) other_robot_live = other_robot_live|(0x01<<2);
//			else other_robot_live=other_robot_live & (~0x04);
//		if(game_robot_HP.red_4_robot_HP>0) other_robot_live = other_robot_live|(0x01<<3);
//			else other_robot_live=other_robot_live & (~0x08);		
//		if(game_robot_HP.red_5_robot_HP>0) other_robot_live = other_robot_live|(0x01<<4);
//			else other_robot_live=other_robot_live & (~0x10);
//		if(game_robot_HP.red_outpost_HP>0) other_robot_live = other_robot_live|(0x01<<5);
//			else other_robot_live=other_robot_live & (~0x20);
//		if(game_robot_HP.red_7_robot_HP>0) other_robot_live = other_robot_live|(0x01<<6);
//			else other_robot_live=other_robot_live & (~0x40);
//		if(game_robot_HP.red_base_HP>0) other_robot_live = other_robot_live|(0x01<<7);
//			else other_robot_live=other_robot_live & (~0x80);
//	}
//	else
//		other_robot_live=(0xff-0x20);

		if(need_shoot_flag == 1)
			{
				if(shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value
					&&shoot_control.fric3_ramp.out == shoot_control.fric3_ramp.max_value && shoot_control.fric4_ramp.out == shoot_control.fric4_ramp.max_value)
				{
					shoot_control.shoot_mode = SHOOT_auto_B;
				}
			}
			
//----------------------
//		#endif
	}
    //如果云台状态是 无力状态，就关闭射击(比赛需关闭，TODO)
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control_mode->shoot_mode = SHOOT_STOP;
    }

    shoot_control_mode->last_s = shoot_control_mode->shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

    

/**
 * @brief          设置射击控制量
 * @param[in]      set_control:"shoot_control_t"变量指针.
 * @retval         void
 */
extern power_heat_data_t power_heat_data;

uint16_t  barrel_use=0,last_barrel_use;
uint16_t b_ttt=0,get_z=0;
uint8_t Status_Count1=0;
uint8_t Status_Count2=0;
void shoot_set_control(shoot_control_t *set_control)
{
    if (set_control->shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        set_control->trigger_motor.speed_set = 0.0f;
        set_control->trigger2_motor.speed_set = 0.0f;
    }
    else if (set_control->shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        set_control->trigger_motor.speed_set = 0.0f;
        set_control->trigger2_motor.speed_set = 0.0f;
    }
    else if (set_control->shoot_mode == SHOOT_BULLET||set_control->shoot_mode == SHOOT_auto_B)
    {
        set_control->trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        set_control->trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;

				set_control->trigger2_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        set_control->trigger2_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (set_control->shoot_mode == SHOOT_CONTINUE_BULLET)//|| set_control->shoot_mode == SHOOT_auto_B)
    {
		uint16_t last_bullut;
		uint16_t bullut_consume=last_bullut - get_bullet_remaining();
		set_control->heat_add=bullut_consume*10;//50;
		set_control->heat_change=set_control->heat - set_control->last_heat;
        // 枪口热量上限超限判断(SHOOT_CUNTINUE_HEAT: 50)
#if speed_baohu
			//判断有没有枪管超过热量限制
#if TEST_MODE
			if(set_control->heat+Trigger_Lengque+20<set_control->heat_limit)
			{
					if(set_control->heat_add==set_control->last_heat_add)
					{
								set_control->trigger_motor.speed_set=CONTINUE_TRIGGER_SPEED;
								set_control->heat_add=0; 
					}
									//set_control->heat_add=0.0f;
			}

					else
					{	
								set_control->trigger_motor.speed_set=0.0f;			
					}
					if(barrel_use==0)//1				//刚进入打弹环节							//1
							{
								//打完之后没有超过限制,barrel_use一直等于0且热量增加至也一直等于0，他就会一直进这个函数
									set_control->trigger_motor.speed_set=  CONTINUE_TRIGGER_SPEED;
								barrel_use=1;
							}
			
//			if(get_bullet_remaining()==0)
//			{
//					set_control->trigger2_motor.speed_set=0.0f;
//					set_control->trigger_motor.speed_set=0.0f;
//			}
			
				last_bullut=get_bullet_remaining();
        trigger_motor_turn_back();
					set_control->last_heat_add=set_control->heat_add;
				set_control->trigger_motor.last_speed_set=set_control->trigger_motor.speed_set;


#else

        set_control->trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;//后（右）

				last_bullut=get_bullet_remaining();

       trigger_motor_turn_back();// 堵转处理函数
#endif
#endif
 }
}
/**
 * @brief          计算射击数据
 * @param[in]      control_loop:"shoot_control_t"变量指针.
 * @retval         void
 */
void shoot_control_loop(shoot_control_t *control_loop)
{
    if (control_loop->shoot_mode == SHOOT_STOP)
    {
        control_loop->trigger_motor.give_current = 0;
        control_loop->trigger2_motor.give_current = 0;
		
        //摩擦轮需要一个个斜波关闭，不能同时直接关闭，否则可能伤电机
        ramp_calc(&control_loop->fric1_ramp, -SHOOT_FRIC_RAMP_ADD_VALUE);
        ramp_calc(&control_loop->fric2_ramp, -SHOOT_FRIC_RAMP_ADD_VALUE);
				ramp_calc(&control_loop->fric3_ramp, -SHOOT_FRIC_RAMP_ADD_VALUE);
        ramp_calc(&control_loop->fric4_ramp, -SHOOT_FRIC_RAMP_ADD_VALUE);		
    }
    else
    {
        //摩擦轮速度没达标的情况下不开启拨弹电机
        if (control_loop->shoot_mode < SHOOT_READY_BULLET)//枚举
        {
            control_loop->trigger_motor.give_current = 0;
            control_loop->trigger2_motor.give_current = 0;
			
        }
        else
        {
            //计算拨弹轮电机PID
            PID_calc(&control_loop->trigger_motor_pid, control_loop->trigger_motor.speed, control_loop->trigger_motor.speed_set);
            PID_calc(&control_loop->trigger2_motor_pid, control_loop->trigger2_motor.speed, control_loop->trigger2_motor.speed_set);
			
            //给拨弹电机赋值电流
            control_loop->trigger_motor.give_current = (int16_t)(control_loop->trigger_motor_pid.out);
						control_loop->trigger2_motor.give_current =(int16_t)(control_loop->trigger2_motor_pid.out);
        }

      //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
			ramp_calc(&control_loop->fric1_ramp, SHOOT_FRIC_RAMP_ADD_VALUE);
			ramp_calc(&control_loop->fric2_ramp, SHOOT_FRIC_RAMP_ADD_VALUE);
			ramp_calc(&control_loop->fric3_ramp, SHOOT_FRIC_RAMP_ADD_VALUE);
			ramp_calc(&control_loop->fric4_ramp, SHOOT_FRIC_RAMP_ADD_VALUE);		
    }
		
		

    //使用斜波函数计算出的摩擦轮电机转速增加值作为摩擦轮电机转速设定值，缓慢开启和关闭电机
    control_loop->fric_speed1_set = (int16_t)(control_loop->fric1_ramp.out);
    control_loop->fric_speed2_set = (int16_t)(control_loop->fric2_ramp.out);
		control_loop->fric_speed3_set = (int16_t)(control_loop->fric3_ramp.out);
    control_loop->fric_speed4_set = (int16_t)(control_loop->fric4_ramp.out);

    PID_calc(&control_loop->fric_motor_pid[0], control_loop->shoot_motor[0].speed, control_loop->fric_speed1_set);
    PID_calc(&control_loop->fric_motor_pid[2], control_loop->shoot_motor[2].speed, control_loop->fric_speed2_set);
		PID_calc(&control_loop->fric_motor_pid[1], control_loop->shoot_motor[1].speed, control_loop->fric_speed3_set);
    PID_calc(&control_loop->fric_motor_pid[3], control_loop->shoot_motor[3].speed, control_loop->fric_speed4_set);

    control_loop->shoot_motor[0].give_current = control_loop->fric_motor_pid[0].out;
    control_loop->shoot_motor[2].give_current = control_loop->fric_motor_pid[2].out;

		control_loop->shoot_motor[1].give_current = control_loop->fric_motor_pid[1].out;
    control_loop->shoot_motor[3].give_current = control_loop->fric_motor_pid[3].out;

    //UI摩擦轮图形显示，在UI界面上显示摩擦轮转速是否达标
    control_loop->F_speed = (control_loop->shoot_motor[1].speed + control_loop->shoot_motor[0].speed) / 2;
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
fp32 Shoot_Speed_Set=0;
uint8_t barrel_use_Flag=0;
uint16_t  barrel_turn_time=0,Buffer_Time=0;
uint16_t last_bullut=0;
void shoot_bullet_control(void)
{

		uint16_t bullut_consume=last_bullut - get_bullet_remaining();
		shoot_control.heat_add=bullut_consume*10;//50;
    //每次拨动1/10PI的角度，直到射出一颗子弹为止
//    if (shoot_control.shoot_turn_flag == 0)
//    {		//在这里连续对拨弹轮的角度进行改变
//				shoot_control.trigger_motor.angle_set = rad_format(shoot_control.trigger_motor.angle - PI_TEN);
//				shoot_control.trigger2_motor.angle_set = rad_format(shoot_control.trigger2_motor.angle + PI_TEN);
//        shoot_control.shoot_turn_flag = 1;
//    }

//    if (rad_format(shoot_control.trigger2_motor.angle_set - shoot_control.trigger2_motor.angle) > 0.05f
//			||rad_format(shoot_control.trigger_motor.angle_set - shoot_control.trigger_motor.angle) < - 0.05f)
    {
			//如果在前哨战，就一颗一颗的打子弹
//			if(auto_receive.auto_position==1)
//			{
//					Shoot_Speed_Set = TRIGGER_SPEED;
//			}
//			else
//			{
//					Shoot_Speed_Set = CONTINUE_TRIGGER_SPEED;//拨弹轮转速
//			}
			
//			if(shoot_control.heat1+Trigger_Lengque < shoot_control.heat_limit&&shoot_control.heat2+Trigger_Lengque < shoot_control.heat_limit)
//			{
//					//判断弹舱是否还有子弹(单边)
//				//为什么不行？子弹的剩余量现在我们得不到
//					if(shoot_control.heat_add==shoot_control.last_heat_add)
//					{
//							
//						//判断之前是哪根枪管打子弹，换边
//							if(barrel_use_Flag==1)//6				//枪管1没子弹了     								//3
//							{
//								shoot_control.trigger_motor.speed_set=0.0f;
//								shoot_control.trigger2_motor.speed_set=  Shoot_Speed_Set;
//								//进来之后
//								shoot_control.heat_add=0; 
//								barrel_use_Flag=2;
//								//因为进来之后换了枪管，热量数据有延时，所以下一刻又会进入这个判断，barre_use=1,
//							}
//							else	if(barrel_use_Flag==2)//7		
//							{
//								shoot_control.trigger_motor.speed_set= Shoot_Speed_Set;
//								shoot_control.trigger2_motor.speed_set=0.0f;
//								shoot_control.heat_add=0;
//								barrel_use_Flag=1;
//							}
//							else if(barrel_use_Flag==0)//1				//刚进入打弹环节							//1
//							{
//								//打完之后没有超过限制,barrel_use一直等于0且热量增加至也一直等于0，他就会一直进这个函数
//									shoot_control.trigger_motor.speed_set=  Shoot_Speed_Set;
//									shoot_control.trigger2_motor.speed_set=0.0f;
//								//因为代码运行速度很快，一进来就给barrel_use赋值为0，然后还没打出子弹，热量又还是为0，就换换成另一边打弹
////								if(barrel_turn_time>1000)
////								{
//										barrel_use_Flag=1;
////										barrel_turn_time=0;
////								}
////								else barrel_turn_time++;
//						}
//					}
//					//两边弹舱都有子弹
//					else
//					{	
//						//判断应该让哪边枪管打子弹
//						if(barrel_use_Flag==1)//2						//一直打没超过热量限制					//2
//							{
//								shoot_control.trigger_motor.speed_set =  Shoot_Speed_Set;
//								shoot_control.trigger2_motor.speed_set =0.0f;
//							}
//							else if(barrel_use_Flag==2)//4			//枪管2一直没超过热量限制						//4
//							{
//								shoot_control.trigger_motor.speed_set=0.0f;
//								shoot_control.trigger2_motor.speed_set= Shoot_Speed_Set;
//							}			
//					}
//			}
			//在这个判断里肯定只有一个枪管不超热量
//			else
//			{
//				//判断是哪个枪管超热量
//				//尽管热量下降的不是那么快，但是热量上限值一直大于限制，会一直在这个函数里执行，当热量下降到某个值，进入另一个热量限制判断
//					if(shoot_control.heat1+Trigger_Lengque>=shoot_control.heat_limit)//3		//超过热量限制
//					{
//							shoot_control.trigger_motor.speed_set = 0.0f;
//							shoot_control.trigger2_motor.speed_set =  Shoot_Speed_Set;
//							barrel_use_Flag=2;																					//枪管1超热量
//					}
//					else if(shoot_control.heat2+Trigger_Lengque>=shoot_control.heat_limit)//5		//枪管2超过热量限制
//					{						
//						shoot_control.trigger_motor.speed_set = Shoot_Speed_Set;
//						shoot_control.trigger2_motor.speed_set=0.0f;
//						barrel_use_Flag=1;																						//枪管2超热量
//					}
///*else*/ if(shoot_control.heat2+Trigger_Lengque>=shoot_control.heat_limit&&
//						shoot_control.heat1+Trigger_Lengque>=shoot_control.heat_limit)//保护代码
//					{
//						shoot_control.trigger_motor.speed_set=0.0f;
//						shoot_control.trigger2_motor.speed_set=0.0f;
//						//当热量低于某一阈值再让barrel_use=0开启打弹1
//						barrel_use_Flag=3;																						//两个枪管都超热量(等待状态)
//						//小于某一个值的时候使标志位等于0，重新可以打弹
//						if(shoot_control.heat2+Trigger_Lengque<70&&shoot_control.heat1+Trigger_Lengque<70)
//						{
//							barrel_use_Flag=0;
//						}
//					}
//			}
//			if(get_bullet_remaining()==0)//8
//			{		//没弹后关闭拨弹轮
//					shoot_control.trigger_motor.speed_set=0.0f;
//					shoot_control.trigger2_motor.speed_set=0.0f;
//			}
       			if(shoot_control.heat+Trigger_Lengque+22<shoot_control.heat_limit)
			{
					if(shoot_control.heat_add==shoot_control.last_heat_add)
					{
								shoot_control.trigger_motor.speed_set=CONTINUE_TRIGGER_SPEED;
																																																																																												shoot_control.heat_add=0; 
					}
									//set_control->heat_add=0.0f;
			}

					else
					{	
								shoot_control.trigger_motor.speed_set=0.0f;			
					}
					if(barrel_use==0)//1				//刚进入打弹环节							//1
							{
								//打完之后没有超过限制,barrel_use一直等于0且热量增加至也一直等于0，他就会一直进这个函数
									shoot_control.trigger_motor.speed_set=  CONTINUE_TRIGGER_SPEED;
								barrel_use=1;
							}
			
//			if(get_bullet_remaining()==0)
//			{
//					shoot_control.trigger2_motor.speed_set=0.0f;
//					shoot_control.trigger_motor.speed_set=0.0f;
//			}
			
				last_bullut=get_bullet_remaining();
        trigger_motor_turn_back();
					shoot_control.last_heat_add=shoot_control.heat_add;
				shoot_control.trigger_motor.last_speed_set=shoot_control.trigger_motor.speed_set;
			
			
			
//			if(shoot_control.trigger_motor.speed_set ==Shoot_Speed_Set)//持续射击
//			{
//					trigger_motor_turn_back();//堵转处理
//			}
//			if(shoot_control.trigger2_motor.speed_set ==Shoot_Speed_Set)
//			{
//					trigger2_motor_turn_back();
//			}
    }
//    else    // 没达到目标角度
//    {
//        shoot_control.shoot_turn_flag = 0;
//    }
//		last_bullut=get_bullet_remaining();//实时记录剩余弹量
		if(Buffer_Time>=1000)
		{//到了一定时间后记录热量增加值
			shoot_control.last_heat_add=shoot_control.heat_add;
			Buffer_Time=0;
		}			
		else Buffer_Time++;
//								shoot_control.trigger_motor.speed_set=Shoot_Speed_Set;
//								shoot_control.trigger2_motor.speed_set=  Shoot_Speed_Set;
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */

// 堵转处理函数
void trigger_motor_turn_back(void)
{			
//			//枪管2超热量，枪管1卡弹
//			if(shoot_control.turn_back_flag==OVERHEAT2_TURNBACK1)
//			{
//				//不论是不是让枪管1打弹，只要枪管2的热量大于200就让他执行堵转程序
//					if(shoot_control.heat2+Trigger_Lengque>200&&shoot_control.trigger2_motor.speed_set==0.0f
//						&&shoot_control.heat1+Trigger_Lengque<shoot_control.heat_limit)
//					{
//						//说明不卡弹了
//						shoot_control.trigger_motor.speed_set = CONTINUE_TRIGGER_SPEED;
//						//如果此时让枪管2正常打弹，但是热量大于了200却没超过热量限制，那么就不能用热量添加值去判断了
//						if(shoot_control.heat_add!=0)
//							shoot_control.turn_back_flag=NO_TURNBACK;
//					}
//			}

//    //拨弹电机堵转判断，达到堵转时间就反向旋转电机
//    if (shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.trigger_motor.speed_set = shoot_control.trigger_motor.speed_set;
//    }
//    else
//    {
//        shoot_control.trigger_motor.speed_set = -shoot_control.trigger_motor.speed_set;
//    }	

//    //电机堵转卡弹计时
//    if (fabs(shoot_control.trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.block_time++;
//        shoot_control.reverse_time = 0;
//    }
//    //电机堵转卡弹反转计时
//    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//    {
//        shoot_control.reverse_time++;
//    }
//    else
//    {
//        shoot_control.block_time = 0;
//    }
   //拨弹电机堵转判断，达到堵转时间就反向旋转电机
    if (shoot_control.block_time < BLOCK_TIME)//没到卡弹时间
    {
        shoot_control.trigger_motor.speed_set = shoot_control.trigger_motor.speed_set;
    }
    else//到了卡弹时间
    {
        shoot_control.trigger_motor.speed_set = -shoot_control.trigger_motor.speed_set;
    }

    //电机堵转卡弹计时
    if (fabs(shoot_control.trigger_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {//拨弹轮转速不达标进入堵转卡弹计时
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    //电机堵转卡弹反转计时
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {//卡弹计时达标且反转计时未达标
        shoot_control.reverse_time++;
    }
    else
    {//反转完后计时清零
        shoot_control.block_time = 0;
    }
}

void trigger2_motor_turn_back(void)
{
//			if(shoot_control.turn_back_flag==OVERHEAT1_TURNBACK2)
//			{
//					if(shoot_control.heat1+Trigger_Lengque>200&&shoot_control.trigger_motor.speed_set==0.0f
//						&&shoot_control.heat2+Trigger_Lengque<shoot_control.heat_limit)
//					{
//						shoot_control.trigger2_motor.speed_set = - CONTINUE_TRIGGER_SPEED;
//						if(shoot_control.heat_add!=0)
//							shoot_control.turn_back_flag=NO_TURNBACK;
//					}
//			}
//		
//		
//    //拨弹电机堵转判断，达到堵转时间就反向旋转电机
//    if (shoot_control.block_time2 < BLOCK_TIME)
//    {
//        shoot_control.trigger2_motor.speed_set =shoot_control.trigger2_motor.speed_set;
//    }
//    else
//    {
//        shoot_control.trigger2_motor.speed_set =-shoot_control.trigger2_motor.speed_set;
//    }

//    //电机堵转卡弹计时
//    if (fabs(shoot_control.trigger2_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time2 < BLOCK_TIME)
//    {
//        shoot_control.block_time2++;
//        shoot_control.reverse_time2 = 0;
//    }
//    //电机堵转卡弹反转计时
//    else if (shoot_control.block_time2 == BLOCK_TIME && shoot_control.reverse_time2 < REVERSE_TIME)
//    {
//        shoot_control.reverse_time2++;
//    }
//    else
//    {
//        shoot_control.block_time2 = 0;
//    }
   if (shoot_control.block_time2 < BLOCK_TIME)
    {
        shoot_control.trigger2_motor.speed_set =shoot_control.trigger2_motor.speed_set;
    }
    else
    {
        shoot_control.trigger2_motor.speed_set =-shoot_control.trigger2_motor.speed_set;//反转
    }

    //电机堵转卡弹计时
    if (fabs(shoot_control.trigger2_motor.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time2 < BLOCK_TIME)
    {
        shoot_control.block_time2++;
        shoot_control.reverse_time2 = 0;
    }
    //电机堵转卡弹反转计时
    else if (shoot_control.block_time2 == BLOCK_TIME && shoot_control.reverse_time2 < REVERSE_TIME)
    {
        shoot_control.reverse_time2++;
    }
    else
    {
        shoot_control.block_time2 = 0;
    }
}


//#if J_scope_TEST_MODE
//int32_t trigger_motor_speed_1000, trigger_motor_speed_set_1000;
//int32_t shoot_motor_speed1_1000, shoot_motor_speed1_set_1000;
//int32_t shoot_motor_speed2_1000, shoot_motor_speed2_set_1000;
//static void J_scope_test(void)
//{
//    trigger_motor_speed_1000 = (int32_t)(shoot_control.trigger_motor.speed * 1000);
//    trigger_motor_speed_set_1000 = (int32_t)(shoot_control.trigger_motor.speed_set * 1000);

//    shoot_motor_speed1_1000 = (int32_t)(shoot_control.shoot_motor[0].speed * 1000);
//    shoot_motor_speed1_set_1000 = (int32_t)(shoot_control.fric_speed1_set * 1000);

//    shoot_motor_speed2_1000 = (int32_t)(shoot_control.shoot_motor[1].speed * 1000);
//    shoot_motor_speed2_set_1000 = (int32_t)(shoot_control.fric_speed2_set * 1000);
//}
//#endif


