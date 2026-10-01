/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled∆¡ƒªœ‘ æ¥ÌŒÛ¬Î
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef OLED_TASK_H
#define OLED_TASK_H
#include "struct_typedef.h"



/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled»ŒŒÒ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void oled_task(void const * argument);



#include "bsp_buzzer.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot.h"

//extern void TemperatureProtection_Task(void const *pvParameters);
	
#define TEMPERATURE 60

#define MAX_PAGE 2 
#define MAX_PRESH_TIME 5 


//typedef struct
//{
//     const motor_measure_t *gimbal_yaw_motor;
//	 const motor_measure_t *gimbal_pitch_motor;
//	 const motor_measure_t *chassis_motor[4];
//	 const motor_measure_t *trigger_motor;
//	 const motor_measure_t *friction_motor[2];
//	 
//	 uint8_t yaw_motor_flag;
//	 uint8_t pitch_motor_flag;
//	 uint8_t chassis_motor_flag[4];
//	 uint8_t trigger_motor_flag;
//	 uint8_t friction_motor_flag[2];
//	 
//	 uint16_t buzzer_on_flag;
//	 
//	 uint16_t buzzer_time[9];
//	
//}Temperature_t;

//extern void gimbal_motor_TemperatureProtection(void);
//extern void chassis_motor_TemperatureProtection(void);
//extern void friction_motor_TemperatureProtection(void);


#endif
