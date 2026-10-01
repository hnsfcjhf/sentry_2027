/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled屏幕显示错误码
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
#include "oled_task.h"
#include "main.h"
#include "oled.h"

#include "cmsis_os.h"
#include "detect_task.h"
#include "voltage_task.h"

#include "MainFocus_task.h"
#include "gimbal_behaviour.h"

#define OLED_CONTROL_TIME 30
#define REFRESH_RATE    10

const error_t *error_list_local;

uint8_t other_toe_name[4][4] = {"GYR\0","ACC\0","MAG\0","REF\0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;

//Temperature_t Temperature;
extern gimbal_control_t gimbal_control;
//extern gimbal_behaviour_e gimbal_behaviour;
extern chassis_move_t chassis_move;
//static void TemperatureProtection_Init();

//----------------------------------------
uint8_t page_num=1;
uint8_t key_presh_time=0;
uint8_t key_presh_flag=0;



//----------------------------------------

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
    uint8_t i;
    uint8_t show_col, show_row;
    error_list_local = get_error_list_point();
    osDelay(1000);
    OLED_init();
    OLED_LOGO();
    i = 100;
    while(i--)
    {
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
    
//    TemperatureProtection_Init();
    
    while(1)
    {
        //use i2c ack to check the oled
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }

        now_oled_errror = toe_is_error(OLED_TOE);
        //oled init
        if(last_oled_error == 1 && now_oled_errror == 0)
        {
            OLED_init();
        }

        if(now_oled_errror == 0)
        {
            refresh_tick++;
            //10Hz refresh
            if(refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE))
            {
                refresh_tick = 0;
                OLED_operate_gram(PEN_CLEAR);
//                OLED_show_graphic(0, 1, &battery_box);

//                if(get_battery_percentage() < 10)
//                {
//                    OLED_printf(9, 2, "%d", get_battery_percentage());
//                }
//                else if(get_battery_percentage() < 100)
//                {
//                    OLED_printf(6, 2, "%d", get_battery_percentage());
//                }
//                else
//                {
//                    OLED_printf(3, 2, "%d", get_battery_percentage());
//                }

               				//gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->temperate
				
				
				

				
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)==0)//KEY按键按下切换页面
				{
					key_presh_time++;
					if(key_presh_time>MAX_PRESH_TIME)//按键消抖
					{
						page_num++;
						key_presh_time=0;
					}
					if(page_num>MAX_PAGE)
					{
						page_num=1;
					}
				}
				
				
				
				switch(page_num)
				{
					case 1: 
					{
//						OLED_operate_gram(PEN_CLEAR);
//						
						OLED_show_string(0, 0, "auto_x:");
						OLED_printf(45, 0, "%.2f", auto_receive.auto_x);
						
						OLED_show_string(0, 12, "auto_y:");
						OLED_printf(45, 12, "%.2f", auto_receive.auto_y);


						OLED_show_string(0, 24, "YY:");
						OLED_printf(20, 24, "%.3f", auto_receive.auto_YY);

						OLED_show_string(0, 36, "PP:");
						OLED_printf(20, 36 ,"%.3f", auto_receive.auto_PP);

									
						OLED_show_string(0, 48, "aim_flag:");
						OLED_printf(55, 48 ,"%d", auto_receive.vision_flag);
						
						//---------------------------
						OLED_show_string(75, 12, "camp:");
						OLED_printf(105,12,"%c", auto_manage.auto_send.camp);


//						OLED_show_string(95, 12, "P_M");
//						OLED_show_graphic(115, 12, &check_box[error_list_local[PITCH_GIMBAL_MOTOR_TOE].error_exist]);

						OLED_show_string(90, 24, "DBUS");
						OLED_show_graphic(115, 24, &check_box[error_list_local[DBUS_TOE].error_exist]);
						
						OLED_show_string(75, 36, "Gi_MODE:");
						OLED_printf(120, 36 ,"%d", gimbal_behaviour);

						OLED_show_string(75, 48, "ch_MODE:");
						OLED_printf(120, 48 ,"%d", chassis_move.chassis_mode);
						
						break;
					}
					case 2:
					{
//						OLED_operate_gram(PEN_CLEAR);
						  for(i = CHASSIS_MOTOR1_TOE; i < REFEREE_TOE + 1; i++)//1-8
							{
								show_col = ((i-1) * 32) % 128;//列
								show_row = (i-1) / 4 * 12;//30 + (i-1) / 4 * 12;//行
								OLED_show_char(show_col, show_row, 'M');//0-96（x） 0-12 （y）
								OLED_show_char(show_col + 6, show_row, '0'+i);
								OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
								
								//-----------------------------------------------------------------------------------------
											
							}
							
							OLED_show_string(28, 36, "yaw_T:");
							OLED_printf(62, 36 ,"%d", gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->temperate);
							OLED_show_string(75, 36, "pit_T:");
							OLED_printf(110, 36 ,"%d", gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->temperate);
							
							OLED_show_string(0, 48, "base_hp:");
							OLED_printf(50, 48 ,"%d", get_base_HP());
							OLED_show_string(65, 48, "rfid_new:");
							OLED_printf(120, 48 ,"%d", get_rfid_status());

						
						break;
					}
					case 3:
					{
						
					}
//					default:
//					{
//						
//						break;
//					}					
				}
					
//				OLED_show_string(90, 24, "DBUS");
//                OLED_show_graphic(115,24, &check_box[error_list_local[DBUS_TOE].error_exist]);


//                OLED_show_string(80, 12, "YY:");
//                OLED_printf(100, 12, "%d", auto_receive.auto_YY);
				//---------------------------------------------------------------------------------------------------
								
//				OLED_show_string(80, 18, "Aim_x:");
//                OLED_printf(104, 18, "%d", gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->temperate);
//				
//				OLED_show_string(80, 18, "Aim_y:");
//                OLED_printf(104, 18, "%d", gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->temperate);




//                for(i = CHASSIS_MOTOR1_TOE; i < TRIGGER_MOTOR_TOE + 1; i++)
//                {
//                    show_col = ((i-1) * 32) % 128;
//                    show_row = 15 + (i-1) / 4 * 12;
//                    OLED_show_char(show_col, show_row, 'M');
//                    OLED_show_char(show_col + 6, show_row, '0'+i);
//                    OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
//                }

//                for(i = BOARD_GYRO_TOE; i < REFEREE_TOE + 1; i++)
//                {
//                    show_col = (i * 32) % 128;
//                    show_row = 15 + i / 4 * 12;
//                    OLED_show_string(show_col, show_row, other_toe_name[i - BOARD_GYRO_TOE]);
//                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);
//                }

                OLED_refresh_gram();
            }
        }



        last_oled_error = now_oled_errror;
	} 
        
} 



//--------------------------------温控保护----------------------------------------------------------------------------------------------		
//        if(Temperature.gimbal_yaw_motor->temperate > TEMPERATURE && Temperature.gimbal_yaw_motor->temperate < 65.0f)
//		 {
//			 Temperature.yaw_motor_flag = 1;
//			 Temperature.buzzer_on_flag = Temperature.buzzer_time[4];
//		 } 

//		 if(Temperature.gimbal_pitch_motor->temperate > TEMPERATURE && Temperature.gimbal_pitch_motor->temperate < 65.0f)
//		 {
//			 Temperature.pitch_motor_flag = 1;
//			 Temperature.buzzer_on_flag = Temperature.buzzer_time[5];
//		 } 
//		 
//		 if(Temperature.trigger_motor->temperate > TEMPERATURE && Temperature.trigger_motor->temperate < 65.0f)
//		 {
//			 Temperature.trigger_motor_flag = 1;
//			 Temperature.buzzer_on_flag = Temperature.buzzer_time[6];
//		 }

//		 for(int i = 0; i < 2; i++)
//		 {
//				 if(Temperature.friction_motor[i]->temperate > TEMPERATURE && Temperature.friction_motor[i]->temperate < 65.0f)
//				 {
//					 Temperature.friction_motor_flag[i] = 1;
//					 Temperature.buzzer_on_flag = Temperature.buzzer_time[7 + i];
//				 }
//	   }
//		 
//		 for (int j = 0; j < 4; j++)
//     {
//         if(Temperature.chassis_motor[0]->temperate > TEMPERATURE && Temperature.chassis_motor[0]->temperate < 65.0f)
//		     {
//			     Temperature.chassis_motor_flag[j] = 1;
//					 Temperature.buzzer_on_flag = Temperature.buzzer_time[j];
//		     }
//     }
//		 
////		 if(Temperature.buzzer_on_flag != 0)
////		 {
////            buzzer_on(840, 300);
////            HAL_Delay(Temperature.buzzer_on_flag);
////		    buzzer_on(84, 300);
////		    HAL_Delay(7000);			
////		 }


//        osDelay(OLED_CONTROL_TIME);
//    }
//}

/////////////////////////////////////////////////////////////////////////////////////

//void TemperatureProtection_Init()
//{
//    Temperature.gimbal_yaw_motor = get_yaw_gimbal_motor_measure_point(); 
//    Temperature.gimbal_pitch_motor = get_pitch_gimbal_motor_measure_point(); 
//	  Temperature.trigger_motor = get_trigger_motor_measure_point();
//    for (int i = 0; i < 2; i++)
//    {   
//        Temperature.friction_motor[i] = get_shoot_motor_measure_point(i);
//    }

//		for (int j = 0; j < 4; j++)
//    {
//        Temperature.chassis_motor[j] = get_chassis_motor_measure_point(j);
//    }
//		
//		
//		
//		Temperature.yaw_motor_flag = 0;
//		Temperature.pitch_motor_flag = 0;
//		Temperature.trigger_motor_flag = 0;
//		
//		for (int k = 0; k < 2; k++)
//    {   
//        Temperature.friction_motor_flag[k] = 0;
//    }
//		
//		for (int l = 0; l < 4; l++)
//    {
//        Temperature.chassis_motor_flag[l] = 0;
//    }
//		
//		Temperature.buzzer_on_flag = 0;
//		
//		Temperature.buzzer_time[0] = 500; 
//		Temperature.buzzer_time[1] = 1000;
//		Temperature.buzzer_time[2] = 2000;
//		Temperature.buzzer_time[3] = 2500; 
//		Temperature.buzzer_time[4] = 3000;
//		Temperature.buzzer_time[5] = 3500;
//		Temperature.buzzer_time[6] = 4000; 
//		Temperature.buzzer_time[7] = 5000;
//		Temperature.buzzer_time[8] = 5500;
//}


//void gimbal_motor_TemperatureProtection()
//{
//   if(Temperature.yaw_motor_flag == 1)
//	 {
//	    gimbal_control.gimbal_yaw_motor.given_current = 0;
//	 }

//   if(Temperature.pitch_motor_flag == 1)
//	 {
//	    gimbal_control.gimbal_pitch_motor.given_current = 0;
//	 }
//	 
////	 if(Temperature.trigger_motor_flag == 1||Temperature.friction_motor_flag[0] == 1||Temperature.friction_motor_flag[1] == 1)
////	 {
////	    shoot_control.given_current = 0;
////	 }
//	 
//}

//void chassis_motor_TemperatureProtection()
//{
//   if(Temperature.chassis_motor_flag[0] == 1)
//	 {
//	    chassis_move.motor_chassis[0].give_current = 0;
//	 }

//   if(Temperature.chassis_motor_flag[1] == 1)
//	 {
//	    chassis_move.motor_chassis[1].give_current = 0;
//	 }
//	 
//	 if(Temperature.chassis_motor_flag[2] == 1)
//	 {
//	    chassis_move.motor_chassis[2].give_current = 0;
//	 }
//	 
//	 if(Temperature.chassis_motor_flag[3] == 1)
//	 {
//	    chassis_move.motor_chassis[3].give_current = 0;
//	 }
//	 
//}

////void friction_motor_TemperatureProtection()
////{
////   if(Temperature.friction_motor_flag[0] == 1||Temperature.friction_motor_flag[1] == 1)
////	 {
////	    shoot_control.shoot_speed_pid[0].out = 0;
////		  shoot_control.shoot_speed_pid[1].out = 0;
////	 }

////}

