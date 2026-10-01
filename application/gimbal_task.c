/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "usart.h"
#include "MainFocus_task.h"
#include "chassis_task.h"
//#include "Auto_attackTask.h"


// 电机编码值规整 0—8191
#define ecd_format(ecd)    \
  {                        \
    if ((ecd) > ECD_RANGE) \
      (ecd) -= ECD_RANGE;  \
    else if ((ecd) < 0)    \
      (ecd) += ECD_RANGE;  \
  }

// 云台所有PID清除
#define gimbal_total_pid_clear(gimbal_clear)                                                    \
  {                                                                                             \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);        \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_auto);   \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);        \
    PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                         \
                                                                                                \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);      \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid_auto); \
    gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);      \
    PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                       \
  }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

uint8_t temp_losed_flag=0;
uint8_t rc_flag=1;
  
/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台自瞄控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control_auto(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机(0)或者pitch电机(1)
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, fp32 P_motor);//fp32 last_ab_angle, fp32 P_motor);

/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          云台角度PID初始化, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 * @param[out]     pid:云台PID指针
 * @param[in]      maxout: pid最大输出
 * @param[in]      intergral_limit: pid最大积分输出
 * @param[in]      kp: pid kp
 * @param[in]      ki: pid ki
 * @param[in]      kd: pid kd
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);

/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

/**
 * @brief          云台角度PID计算, 因为角度范围在(-pi,pi)，不能用PID.c的PID
 * @param[out]     pid:云台PID指针
 * @param[in]      get: 角度反馈
 * @param[in]      set: 角度设定
 * @param[in]      error_delta: 角速度
 * @retval         pid 输出
 */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

/**
 * @brief          云台校准计算
 * @param[in]      gimbal_cali: 校准数据
 * @param[out]     yaw_offset:yaw电机云台中值
 * @param[out]     pitch_offset:pitch 电机云台中值
 * @param[out]     max_yaw:yaw 电机最大机械角度
 * @param[out]     min_yaw: yaw 电机最小机械角度
 * @param[out]     max_pitch: pitch 电机最大机械角度
 * @param[out]     min_pitch: pitch 电机最小机械角度
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

#if J_scope_TEST_MODE
// j-scope 帮助pid调参
//static void J_scope_test(void);
// 针对fp32（float）类型的符号函数实现
static int sgn2(fp32 x) ;
#endif

// 云台控制所有相关数据
gimbal_control_t gimbal_control;

// 发送的电机电流
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;

/**
  * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // 云台初始化
    gimbal_init(&gimbal_control);
//    // 射击初始化
//    shoot_init();
    // 判断电机是否都上线
    //        while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE)||toe_is_error(TRIGGER_MOTOR_TOE))
    //        {
    //                vTaskDelay(GIMBAL_CONTROL_TIME);
    //                gimbal_feedback_update(&gimbal_control);                         //云台数据反馈
    //        }

    while (1)
    {
        // 设置云台控制模式
        gimbal_set_mode(&gimbal_control);
        // 控制模式切换 控制数据过渡,用于保持在固定的角度
        gimbal_mode_change_control_transit(&gimbal_control);
        // 云台数据反馈
        gimbal_feedback_update(&gimbal_control);
        // 设置云台控制量
        gimbal_set_control(&gimbal_control);
        // 云台控制PID计算
        gimbal_control_loop(&gimbal_control);
        // 射击任务控制循环
//        shoot_can_set_current = shoot_control_loop();   // 返回的shoot_can_set_current为拨弹轮的电流值
        #if YAW_TURN
                yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
        #else
                yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
        #endif

        #if PITCH_TURN
//			          if (!toe_is_error(DBUS_TOE))
//			          gimbal_control.gimbal_pitch_motor.given_current += 1800;
//			          if(gimbal_control.gimbal_pitch_motor.given_current>=25000) gimbal_control.gimbal_pitch_motor.given_current=25000;
                pitch_can_set_current =-gimbal_control.gimbal_pitch_motor.given_current;
        #else
                pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
        #endif

        //  if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
        //  {

//        if (toe_is_error(DBUS_TOE))
//        {
//            CAN_cmd_gimbal(0, 0, 0, 0);
//        }
//        else
//        {pitch_can_set_current,yaw_can_set_current
//							CAN_cmd_gimbal(0,0 , 0, 0);
//            CAN_cmd_gimbal(1000,1000 , 0, 0);
						            CAN_cmd_gimbal(yaw_can_set_current,pitch_can_set_current , 0, 0);
//						            CAN_cmd_gimbal(0,pitch_can_set_current , 0, 0);
//        }
        // }

        vTaskDelay(GIMBAL_CONTROL_TIME);
//				osDelay(GIMBAL_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    // 云台自瞄PID的KP、KI、KD
    static const fp32 Pitch_speed_pid_auto[3] =  {PITCH_SPEED_PID_KP_AUTO, PITCH_SPEED_PID_KI_AUTO, PITCH_SPEED_PID_KD_AUTO};
    static const fp32 Yaw_speed_pid_auto[3]   =  {YAW_SPEED_PID_KP_AUTO, YAW_SPEED_PID_KI_AUTO, YAW_SPEED_PID_KD_AUTO};
    // 云台PID的KP、KI、KD
    static const fp32 Pitch_speed_pid[3] =  {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3]   =  {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // 电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure    =  get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure  =  get_pitch_gimbal_motor_measure_point();
    // 陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();//欧拉角
    init->gimbal_INT_gyro_point  = get_gyro_data_point();//角速度
    // 遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    // 初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    // 初始化yaw电机pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    // 初始化pitch电机pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    // 初始化 Y 识别 PID
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_auto, YAW_GYRO_ABSOLUTE_PID_MAX_OUT_AUTO, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT_AUTO, YAW_GYRO_ABSOLUTE_PID_KP_AUTO, YAW_GYRO_ABSOLUTE_PID_KI_AUTO, YAW_GYRO_ABSOLUTE_PID_KD_AUTO);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid_auto, PID_POSITION, Yaw_speed_pid_auto, YAW_SPEED_PID_MAX_OUT_AUTO, YAW_SPEED_PID_MAX_IOUT_AUTO);
    // 初始化 P 识别 PID
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid_auto, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT_AUTO, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT_AUTO, PITCH_GYRO_ABSOLUTE_PID_KP_AUTO, PITCH_GYRO_ABSOLUTE_PID_KI_AUTO, PITCH_GYRO_ABSOLUTE_PID_KD_AUTO);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid_auto, PID_POSITION, Pitch_speed_pid_auto, PITCH_SPEED_PID_MAX_OUT_AUTO, PITCH_SPEED_PID_MAX_IOUT_AUTO);

    // 清除所有PID
    gimbal_total_pid_clear(init);

    gimbal_feedback_update(init);
    
    // 初始化妙算数据
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
    
    init->gimbal_pitch_motor.absolute_angle_auto = init->gimbal_pitch_motor.auto_angle = 0;
    init->gimbal_yaw_motor.absolute_angle_auto = init->gimbal_pitch_motor.auto_angle = 0;
		
//		ramp_init(&init->yaw_ramp, GIMBAL_CONTROL_TIME * 0.001f, 0.0033f, -0.0033f);
}


/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}


/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    // yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    // pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}


/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + 1);//陀螺仪俯仰角
#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.offset_ecd);
#else
    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_pitch_motor.offset_ecd);
#endif
    feedback_update->gimbal_pitch_motor.motor_gyro   = -*(feedback_update->gimbal_INT_gyro_point + 1);


    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);//陀螺仪偏航角
#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.offset_ecd);
#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = 
        arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) + 
        arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
		
//		static float yaw_fused = 0.0f;
//static uint8_t first = 1;
//// 在检测到模式从特殊切换到正常时（需要增加状态机）
//static uint8_t last_special = 0;
//uint8_t current_special = (gimbal_behaviour == GIMBAL_AUTO_AIM || auto_receive.auto_revolve==1 || revolve_flag==1);
//if (last_special && !current_special) {
//    // 退出特殊模式，重置滤波器
//    first = 1;
//}
//last_special = current_special;
//if (first) {
//    yaw_fused = feedback_update->gimbal_yaw_motor.absolute_angle;
//    first = 0;
//}

//float dt = 0.001f;   // 1ms
//float gyro_rate = feedback_update->gimbal_yaw_motor.motor_gyro;  // rad/s
//float predicted = yaw_fused + gyro_rate * dt;

//// 计算相对角度与预测值的误差（处理 -π,π 跳变）
//float error = rad_format(feedback_update->gimbal_yaw_motor.relative_angle - predicted);

//// 自适应修正系数：旋转越快，修正越强（加快消除比例误差）
//float K = 0.001f;   // 基础值
//float gyro_abs = fabsf(gyro_rate);
//if (gyro_abs > 0.5f) {  // 旋转速度 > 0.5 rad/s 时增强修正
//    K = 0.005f;
//}

//			if	(gimbal_behaviour == GIMBAL_AUTO_AIM||auto_receive.auto_revolve==1||revolve_flag==1)
//			K = 0.0f;

//yaw_fused = predicted + K * error;
//yaw_fused = rad_format(yaw_fused);

//// 将融合后的角度赋给控制使用的 absolute_angle
//feedback_update->gimbal_yaw_motor.absolute_angle = yaw_fused;
		
}

extern uint8_t auto_flag;   // 自瞄标志
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    
//		static fp32 last_raw_yaw_input = 0.0f;
		
//		if (!(toe_is_error(DBUS_TOE)))
//		{
//		last_raw_yaw_input = add_yaw_angle;
//		ramp_calc(&set_control->yaw_ramp, add_yaw_angle);
//	  add_yaw_angle = set_control->yaw_ramp.out;
//		}
		
    // yaw电机模式控制
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)    //
    { 
        // raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)     //
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle, 0);//last_ab_Y_angle, 0);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

//----------------------------------------------
    // pitch电机模式控制
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)      //
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)      //
    {
        // gyro模式下，陀螺仪角度控制
//        last_ab_P_angle = last_pitch_ab_angle;
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle, 1);//last_ab_P_angle, 1);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}


/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
// 陀螺仪 控制量限制
//extern uint8_t revolve_flag;    // 小陀螺标志

extern uint8_t rc_flag;
extern uint8_t lock_rad;
fp32 add_angle;
int position_time_1;
uint8_t last_upslope=0;
uint8_t upslope_time=0;
extern uint8_t g_fixed;//pitch角度固定值
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, fp32 P_motor)//, fp32 last_ab_angle, fp32 P_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

//	bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
	fp32 angle_set = gimbal_motor->absolute_angle_set-add;

		if(g_fixed==1)//固定值
		{//即直接以add作为absolute_angle_set
			if (P_motor)	//只给pitch轴限位
			{
				if (add  >= gimbal_motor->max_relative_angle)
					add = gimbal_motor->max_relative_angle;
				else if (add  <= gimbal_motor->min_relative_angle)
					add = gimbal_motor->min_relative_angle ;
						else gimbal_motor->absolute_angle_set=add ;
			}
			gimbal_motor->absolute_angle_set = rad_format(add);
		}
		else if(g_fixed==0||rc_flag==1)//偏差值			相对角度
		{
			if(P_motor)//pitch
			{
				if(angle_set>=gimbal_motor->max_relative_angle)
					angle_set=gimbal_motor->max_relative_angle;
				else if(angle_set<=gimbal_motor->min_relative_angle-0.02f)
					angle_set=gimbal_motor->min_relative_angle-0.02f;
				else{
				gimbal_motor->absolute_angle_set= rad_format(angle_set);}
			}
			else//yaw
			{
				//上坡摆正位置（yaw轴角度归零）
				if(auto_receive.auto_upslope==1&&rc_flag==0)
        gimbal_motor->absolute_angle_set=0.0f;//0.0f;
				else if(auto_receive.auto_downslope==1&&rc_flag==0) gimbal_motor->absolute_angle_set=0.0f;
				else 	
					gimbal_motor->absolute_angle_set = rad_format(angle_set);
				
			}
	   last_upslope = auto_receive.auto_upslope;
		
		}
	

}


/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set -= add;
    // 是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}


/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
extern uint64_t losed_flag_time;
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    // yaw轴
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)   //
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)     //
    {

        	if(auto_receive.vision_flag == 0)//未瞄到人
        {
            gimbal_PID_clear(&(control_loop)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid_auto);
            // control_loop->gimbal_yaw_motor.absolute_angle_set=0;
            gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
        }
        else
        {
            gimbal_PID_clear(&(control_loop)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);//云台PID清除，清除pid的out,iout
            gimbal_motor_absolute_angle_control_auto(&control_loop->gimbal_yaw_motor);
					
					if(!switch_is_up(control_loop->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
					{
						losed_flag_time=1;
					}
        }
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }


    // pitch轴
    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)     //
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)   //
    {
			//首先没开自瞄，会一直时1，其次开自瞄只有在丢失目标的时候会有一瞬间是会清零（因为清零转动前的pid数据），其他情况都会是绝对角度控制
        	if(auto_receive.vision_flag == 0)//未瞄到人
        {
            gimbal_PID_clear(&(control_loop)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid_auto);
            // control_loop->gimbal_pitch_motor.absolute_angle_set=0;
            gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
        }
        else
        {
            gimbal_PID_clear(&(control_loop)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);
            gimbal_motor_absolute_angle_control_auto(&control_loop->gimbal_pitch_motor);
						
					if(!switch_is_up(control_loop->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
					{
						losed_flag_time=1;
					}
//            PP_set = control_loop->gimbal_pitch_motor.absolute_angle_set;
//            PP_set_T = control_loop->gimbal_pitch_motor.absolute_angle;delete_y
        }
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
}


/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
//uint32_t l;
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
//		l++;
//		if (l<=5000) gimbal_control.gimbal_pitch_motor.absolute_angle_set=0.2;
//		else gimbal_control.gimbal_pitch_motor.absolute_angle_set=0.4;
//		if (l==10000)l=0;
    // 角度环，速度环串级pid调试（外环的输出值做内环的输入值），外环的输出值越小说明误差越小，速度环的输入值就越小，导致误差越小速度越小越接近目标值
		//以电机当前角速度作为微分前馈，提高响应速度
		//外环输出目标角速度
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

		if (gimbal_motor==&gimbal_control.gimbal_yaw_motor){
		if ( gimbal_motor->motor_gyro >= 0.25f || gimbal_motor->motor_gyro <= -0.25f)
    gimbal_motor->current_set += sgn2( gimbal_motor->motor_gyro_set) * 1500;
			else 
				gimbal_motor->current_set += sgn2 (gimbal_motor->motor_gyro_set) * 1500*4*gimbal_motor->motor_gyro;
		}
		
			else	if (gimbal_motor==&gimbal_control.gimbal_pitch_motor){
		if ( gimbal_motor->motor_gyro >= 0.25f || gimbal_motor->motor_gyro <= -0.25f)
    gimbal_motor->current_set += sgn2( gimbal_motor->motor_gyro_set) * 1200;
			else 
				gimbal_motor->current_set += sgn2 (gimbal_motor->motor_gyro_set) * 1200*4*gimbal_motor->motor_gyro;
		}
		if (gimbal_motor->current_set > 25000)  gimbal_motor->current_set = 25000;
		else if (gimbal_motor->current_set < -25000) gimbal_motor->current_set = -25000;
		    // 控制值赋值
        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control_auto(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    // 角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid_auto,
                                        gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set,
                                                                             gimbal_motor->motor_gyro);
		
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid_auto, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set); //- forwardfeed(gimbal_motor->motor_gyro,1);

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
		   if (gimbal_motor==&gimbal_control.gimbal_yaw_motor)
{		    static fp32 last_angle_set = 0.0f;
    
    // 计算角度误差
    fp32 angle_error = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    
    // 角度环，速度环串级pid调试
    fp32 angle_pid_output = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, 
                                           gimbal_motor->absolute_angle, 
                                           gimbal_motor->absolute_angle_set,
                                           gimbal_motor->motor_gyro);
    
    // 计算目标角速度前馈：通过目标角度的变化率估算
    // dt是控制周期，单位秒（GIMBAL_CONTROL_TIME / 1000.0f）
    fp32 dt = GIMBAL_CONTROL_TIME / 1000.0f;
    fp32 target_angular_velocity = 0.0f;
    
    if (dt > 0.001f) // 避免除零
    {
        // 计算目标角度的变化率（近似为目标角速度）
        target_angular_velocity = rad_format(gimbal_motor->absolute_angle_set - last_angle_set) / dt;
    }
    
    // 保存当前目标角度，用于下一次计算
    last_angle_set = gimbal_motor->absolute_angle_set;
    
    // 速度前馈：将估算的目标角速度加到速度设定值上
    fp32 velocity_feedforward = target_angular_velocity * 0.8f; // 前馈系数
    
    // 速度设定值 = 角度环输出 + 速度前馈
    gimbal_motor->motor_gyro_set = angle_pid_output + velocity_feedforward;
    
    // 速度环计算电流
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, 
                                         gimbal_motor->motor_gyro, 
                                         gimbal_motor->motor_gyro_set);
//				if (gimbal_motor==&gimbal_control.gimbal_pitch_motor){
//		if ( gimbal_motor->motor_gyro >= 0.2f || gimbal_motor->motor_gyro <= -0.2f)
//    gimbal_motor->current_set += sgn2( gimbal_motor->motor_gyro_set) * 3000;
//			else 
//				gimbal_motor->current_set += sgn2 (gimbal_motor->motor_gyro_set) * 3000*5*gimbal_motor->motor_gyro;
//		}
    
    // 控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);}
}


static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    // 角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    // 控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}



/**
 * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
 * @param[in]      yaw_offse:yaw 中值
 * @param[in]      pitch_offset:pitch 中值
 * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
 * @param[in]      min_yaw:yaw 最小相对角度
 * @param[in]      max_yaw:pitch 最大相对角度
 * @param[in]      min_yaw:pitch 最小相对角度
 * @retval         返回空
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
 */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        // 保存进入时候的数据，作为起始数据，来判断最大，最小值
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
		
		
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;///yxc
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
 * @brief          云台校准计算，将校准记录的中值,最大 最小值
 * @param[out]     yaw 中值 指针
 * @param[out]     pitch 中值 指针
 * @param[out]     yaw 最大相对角度 指针
 * @param[out]     yaw 最小相对角度 指针
 * @param[out]     pitch 最大相对角度 指针
 * @param[out]     pitch 最小相对角度 指针
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else
		
		//编码值是一直累加的按道理来说，但是为了使他成为一个圈就会对编码值进行规整使其在1-8191之间
    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN
		//只有在校准的最后一步会用到这个函数，此时不会出现所想的覆盖现象
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}


/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}


/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}



/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     gimbal_init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

/**
 * @brief          PID计算
 * @param[out]     pid:"gimbal_PID_t"变量指针.
 * @param[out]     get:当前值.
 * @param[out]     set:目标值.
 * @param[out]     error_delta:误差值.
 * @retval         none
 */
static fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}


/**
 * @brief          云台PID清除，清除pid的out,iout
 * @param[out]     gimbal_pid_clear:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }//若不清零会使积分项一直累积
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}


//#if J_scope_TEST_MODE
//int32_t yaw_ins_int_1000, pitch_ins_int_1000;
//int32_t yaw_ins_set_1000, pitch_ins_set_1000;
//int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
//int32_t yaw_speed_int_1000, pitch_speed_int_1000;
//int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
//static void J_scope_test(void)
//{
//    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
//    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
//    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
//    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

//    pitch_ins_int_1000 = -(int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
//    pitch_ins_set_1000 = -(int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
//    pitch_speed_int_1000 = -(int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
//    pitch_speed_set_int_1000 = -(int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
//    pitch_relative_angle_1000 = -(int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
//    pitch_relative_set_1000 = -(int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
//}
// 针对fp32（float）类型的符号函数实现
static int sgn2(fp32 x) {
    if (x > 1e-6) {  // 用极小值1e-6避免浮点精度误差（如x=0.0000001误判为0）
        return 1;
    } else if (x < -1e-6) {
        return -1;
    } else {
        return 0;  // x接近0时返回0
    }
}
//#endif

