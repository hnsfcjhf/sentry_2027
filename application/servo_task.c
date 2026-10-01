/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"

servo_t servo;


uint8_t steering_engine_flag = 0;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    servo.servo_rc = get_remote_control_point();
    
    servo.servo1_open = SERVO1_OPEN;
    servo.servo1_close = SERVO1_CLOSE;

    servo.servo2_open = SERVO2_OPEN;
    servo.servo2_close = SERVO2_CLOSE;

    servo_pwm_set(servo.servo1_open, 3);
    servo_pwm_set(servo.servo2_open, 2);
    steering_engine_flag = 1;

    while (1)
    {
        if (servo.servo_rc->key.v & KEY_PRESSED_OFFSET_C || switch_is_up(servo.servo_rc->rc.s[1]))  // 开
        {
            steering_engine_flag = 1;
            servo_pwm_set(servo.servo1_open, 3);
            servo_pwm_set(servo.servo2_open, 2);
        }
        else if (servo.servo_rc->key.v & KEY_PRESSED_OFFSET_V || switch_is_down(servo.servo_rc->rc.s[1]))   // 关
        {
            steering_engine_flag = 0;
            servo_pwm_set(servo.servo1_close, 3);
            servo_pwm_set(servo.servo2_close, 2);
        }

        osDelay(10);
    }
}


