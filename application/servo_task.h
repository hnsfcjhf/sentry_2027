#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM       500
#define SERVO_MAX_PWM       2500
    
// 舵机1，接口3，左
#define SERVO1_OPEN         1100
#define SERVO1_CLOSE        2150

// 舵机2，接口2，右
#define SERVO2_OPEN         1800
#define SERVO2_CLOSE        900

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V


typedef struct
{
    const RC_ctrl_t *servo_rc;
    
    int32_t servo1_open;
    int32_t servo1_close;
    
    int32_t servo2_open;
    int32_t servo2_close;
    
}servo_t;

extern void servo_task(void const * argument);

#endif
