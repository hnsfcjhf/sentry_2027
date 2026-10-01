#include "MainFocus_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "bsp_usart.h"
#include "usart.h"
#include "string.h"
#include "referee.h"
#include "shoot.h"
#include "Referee_DispatchTask.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern gimbal_control_t gimbal_control;

extern game_robot_HP_t game_robot_HP;
extern event_data_t event_data;
uint8_t count_down_5=0;

auto_receive_t  auto_receive;

extern fp32 last_yaw_angle;
extern fp32 last_pitch_angle;
//extern fp32 last_yaw_ab_angle;
//extern fp32 last_pitch_ab_angle;

extern uint8_t imu_data[48];

int8_t Receive_flag=0;
volatile uint8_t recv_end_flag = 0; //一帧数据接收完成标志
 uint8_t data[36]={0};//接收缓存区
 uint8_t *yui=data;
uint16_t ecd_pitch;
uint16_t ecd_yaw;
 
 fp32 last_pitch_ab_angle=0;//todo
fp32 last_yaw_ab_angle=0;


/*****************自瞄***************/
uint8_t Auto_data_t[8]={0};
float  Shoot_data[2]={0};
float PP=0;//云台pitch角度add
float YY=0;//yaw角度add
uint8_t Lock_flag = 0,shoot_stop_flag = 0,lose_flag = 0;//锁定标志位  丢失标志位
uint32_t lose_count = 0;

/******************巡航***************/

fp32 auto_chassis_x,auto_chassis_y,auto_chassis_wz,auto_vx_init;//wgz
uint8_t chassis_auto_status;//wgz
int8_t test_aim;
/*****************云台手发送坐标***************/
float change_position_x=0;
float change_position_y=0;

uint8_t change_x_array[4]={0};
uint8_t change_y_array[4]={0};

/***********红蓝*********/
 uint8_t referee_data[8];

auto_manage_t auto_manage;



extern robot_interaction_data_t robot_interaction_data;
uint8_t sentry_mode_set;
//---------------------------------------------------------------
//float转uint8_t
void FloatToUint8(uint8_t * char_array,float data)
{
    uint8_t i;
    for(i=0;i<4;i++)
    {
        char_array[i] = ((uint8_t*)(&data))[i];
    }
}

//void Uint16ToUint8(uint8_t * char_array,float data)
//{
//    uint8_t i;
//    for(i=0;i<4;i++)
//    {
//        char_array[i] = ((uint8_t*)(&data))[i];
//    }
//}

void get_my_camp(auto_send_t* auto_send_data)	//获取阵营
{
	switch (get_robot_id())
    {
        case 3:
        case 4:
        case 5:
		case 7:
        {
            auto_send_data->camp = 'r';     // 红方
			referee_data[0] = 'r';
            break;
        }
        case 103:
        case 104:
        case 105:
        case 107:			
        {
            auto_send_data->camp = 'b';     // 蓝方
			referee_data[0] = 'b';
            break;
        }
    }
}
void get_send_data(auto_send_t* auto_send_data)
{
    // 欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad   SEND_RATE：100000
    auto_send_data->euler_angle[0] = *(get_INS_angle_point() + 2) * SEND_RATE;    
    auto_send_data->euler_angle[1] = *(get_INS_angle_point() + 1) * SEND_RATE;
    auto_send_data->euler_angle[2] = *(get_INS_angle_point() + 0) * SEND_RATE;

    // 角速度, 0:x轴, 1:y轴, 2:roll轴 单位 rad/s
    auto_send_data->angle_speed[0] = *(get_gyro_data_point() + 2) * SEND_RATE;
    auto_send_data->angle_speed[1] = *(get_gyro_data_point() + 1) * SEND_RATE;
    auto_send_data->angle_speed[2] = *(get_gyro_data_point() + 0) * SEND_RATE;
	
    // 加速度, 0:x轴, 1:y轴, 2:roll轴 单位 m/s2
    auto_send_data->accel_speed[0] =  *(get_accel_data_point()  + 0) * SEND_RATE;
    auto_send_data->accel_speed[1] =  *(get_accel_data_point()  + 1) * SEND_RATE;
    auto_send_data->accel_speed[2] =  *(get_accel_data_point()  + 2) * SEND_RATE;
	get_my_camp(auto_send_data);//获取阵营
}


/**
 * @brief          发送给视觉的数据整理为串口可发送的数据
 * @param[in]      auto_send_data: 自瞄发送数据结构体指针
 * @retval         none
 */
void send_data_resolve(auto_send_t* auto_send_data)
{
    auto_send_data->send_data[0]  = 0xFF;        // 起始帧
    auto_send_data->send_data[48] = 0xFE;        // 结束帧
    // 校验位
    auto_send_data->send_data[10] = 0x66;       // 欧拉角与角速度分割的校验位
    auto_send_data->send_data[20] = 0x77;       // 角速度与加速度分割的校验位
    auto_send_data->send_data[30] = 0x88;       // 加速度与后面的分割的校验位
    
    // 阵营（红方为0，蓝方为1）
    if (auto_send_data->camp == 'r')//red
        auto_send_data->send_data[31] = 0x00;
    else if (auto_send_data->camp == 'b')//blue
        auto_send_data->send_data[31] = 0x01;

    // 射击模式（装甲板为0，打符为1）
    if (auto_send_data->shoot_mode == 'b')
        auto_send_data->send_data[32] = 0x00;
    else if (auto_send_data->shoot_mode == 'f')
        auto_send_data->send_data[32] = 0x01;


	auto_send_data->send_data[33] = (uint8_t)(get_robot_HP() >> 8)&0xff;  // 前哨战
	auto_send_data->send_data[34] = (uint8_t)get_robot_HP()&0xff;         //哨兵当前血量     	

	if(shoot_control.shoot_17_speed==0)shoot_control.shoot_17_speed=28.0f;
	auto_send_data->send_data[35] =((uint16_t)(shoot_control.shoot_17_speed*100)) >>8&0xff;  // 射速
	auto_send_data->send_data[36] =((uint16_t)(shoot_control.shoot_17_speed*100)) &0xff;         //     	

	auto_send_data->send_data[37]=get_game_progress();//other_robot_live;//机器存活情况
                                                                      //高八位右移至低八位，高八位全补0
	auto_send_data->send_data[38]=(uint8_t)(game_robot_HP.ally_outpost_HP >> 8)&0xff;//前哨战血量High
	auto_send_data->send_data[39]=(uint8_t)game_robot_HP.ally_outpost_HP&0xff; //前哨战血量Low
	                                                                    //高八位补0后取低八位
//-----------------------云台手标点-------------------------------------------------------------------	
	auto_send_data->send_data[40]=0x99;//云台手是否标点
		
	uint8_t position_change_flag=0;
	position_change_flag=get_map_command_case(&change_position_x,&change_position_y);//get_free_target_position(&change_position_x,&change_position_y);
	//x、y不为零则置1
	change_position_x=change_position_x*100;
	change_position_y=change_position_y*100;
  
	
	
	auto_send_data->send_data[41]=position_change_flag;//云台手是（1）否（0）标点
	
	auto_send_data->send_data[42]= ((uint16_t)change_position_x)>>8;//robot_interaction_data.user_data[0];//x高位
	auto_send_data->send_data[43]= (uint16_t)change_position_x;//robot_interaction_data.user_data[1];//x低位
	
	auto_send_data->send_data[44]= ((uint16_t)change_position_y)>>8;//robot_interaction_data.user_data[2];//y高位
	auto_send_data->send_data[45]= (uint16_t)change_position_y;//robot_interaction_data.user_data[3];//y低位
	
	auto_send_data->send_data[46]=(uint8_t)(get_bullet_remaining() >> 8)&0xff;
	auto_send_data->send_data[47]=(uint8_t)get_bullet_remaining()&0xff;//发送剩余可发射弹量
  
	
	
//	auto_send_data->send_data[38] = get_event_data();				   // 获取中心增益点状态 
    
    uint8_t i;
    for (i = 0; i < 3; ++i)
    {
		char euler_H = 0, angular_H = 0, accelerated_H = 0;
		char euler_M = 0, angular_M = 0, accelerated_M = 0;
		char euler_L = 0, angular_L = 0, accelerated_L = 0;
				//欧拉角
		if (auto_send_data->euler_angle[i] < 0)//若为负数
		{
			int32_t _ = -auto_send_data->euler_angle[i] ;//取正数
			euler_L = _ & 0xff;           //低八位
			euler_M = (_ >> 8) & 0xff;    //中八位
			euler_H = (_ >> 16) | 0x80;   //高八位+符号位（最高位置1）
		}
		else
		{
			euler_L = auto_send_data->euler_angle[i]  & 0xff;
			euler_M = (auto_send_data->euler_angle[i]  >> 8) & 0xff;
			euler_H = (auto_send_data->euler_angle[i]  >> 16) & 0xff;
		}
		//角速度
		if (auto_send_data->angle_speed[i]  < 0)
		{
			int32_t _ = -auto_send_data->angle_speed[i] ;
			angular_L = _ & 0xff;
			angular_M = (_ >> 8) & 0xff;
			angular_H = (_ >> 16) | 0x80;
		}
		else
		{
			angular_L = auto_send_data->angle_speed[i]  & 0xff;
			angular_M = (auto_send_data->angle_speed[i]  >> 8) & 0xff;
			angular_H = (auto_send_data->angle_speed[i]  >> 16) & 0xff;
		}
		//加速度
		if (auto_send_data->accel_speed[i] < 0)
		{
			int32_t _ = -auto_send_data->accel_speed[i];
			accelerated_L = _ & 0xff;
			accelerated_M = (_ >> 8) & 0xff;
			accelerated_H = (_ >> 16) | 0x80;
		}
		else
		{
			accelerated_L = auto_send_data->accel_speed[i] & 0xff;
			accelerated_M = (auto_send_data->accel_speed[i] >> 8) & 0xff;
			accelerated_H = (auto_send_data->accel_speed[i] >> 16) & 0xff;
		}
		
		auto_send_data->send_data[i * 3 + 1] = euler_H;
        auto_send_data->send_data[i * 3 + 2] = euler_M;
        auto_send_data->send_data[i * 3 + 3] = euler_L;        
		auto_send_data->send_data[i * 3 + 11] = angular_H;
        auto_send_data->send_data[i * 3 + 12] = angular_M;
        auto_send_data->send_data[i * 3 + 13] = angular_L;		
		auto_send_data->send_data[i * 3 + 21] = accelerated_H;
        auto_send_data->send_data[i * 3 + 22] = accelerated_M;
        auto_send_data->send_data[i * 3 + 23] = accelerated_L;
		    //send_data[1~9]、[11~19]、[21~29]为欧拉角、角速度、加速度
    }  
}

void Speed_Data_int2hex(int vx, int vy, int wz, int yaw)
{
	static uint8_t speed_data[16] = {0xFF,
	0x00, 0x00, 0x00, 0x00,
	0x0F, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0xFE};
	
	int H_vx, L_vx, H_vy, L_vy, H_wz, L_wz, H_yaw, L_yaw;
	
	if(vx >= 0)
	{
		H_vx = vx >> 8;
		L_vx = vx & 0xFF;
	}
	else
	{
		vx = -vx;
		H_vx = (vx >> 8) | 0x80;
		L_vx = vx & 0xFF;
	}
	if(vy >= 0)
	{
		H_vy = vy >> 8;
		L_vy = vy & 0xFF;
	}
	else
	{
		vy = -vy;
		H_vy = (vy >> 8) | 0x80;
		L_vy = vy & 0xFF;
	}
	
	if(wz >= 0)
	{
		H_wz = wz >> 8;
		L_wz = wz & 0xFF;
	}
	else
	{
		wz = -wz;
		H_wz = (wz >> 8) | 0x80;
		L_wz = wz & 0xFF;
	}
	
	if(yaw >= 0)
	{
		H_yaw = yaw >> 8;
		L_yaw = yaw & 0xFF;
	}
	else
	{
		yaw = -yaw;
		H_yaw = (yaw >> 8) | 0x80;
		L_yaw = yaw & 0xFF;
	}
	speed_data[1] = referee_data[0];//红蓝信息
	speed_data[6] = H_yaw;
	speed_data[7] = L_yaw;
	speed_data[8] = H_vy;
	speed_data[9] = L_vy;
	speed_data[10] = H_vx;
	speed_data[11] = L_vx;
	speed_data[12] = H_wz;
	speed_data[13] = L_wz;
	HAL_UART_Transmit_DMA(&huart1,speed_data ,16);
}



int shoot_flag = 0;
fp32 vision_p=0;
fp32 vision_y=0;


char data_set;
void Aoto_data_work(int i)
{
	
	//--------------------------------
		Auto_data_t[0] = data[i+1];
		Auto_data_t[1] = data[i+2];
		Auto_data_t[2] = data[i+3];
		Auto_data_t[3] = data[i+4];
		
		auto_receive.Auto_Cap_U=data[i+6];//超电功率控制
    if(get_game_progress()==4){
		auto_receive.vision_flag = data[i+5];//&0x01;//当前角度与视觉设定角度差标志位
		auto_receive.aim_flag = (data[i+5]&0x01);}
		else {
		auto_receive.vision_flag = 0;
		auto_receive.aim_flag = 0;}
	  test_aim = (int8_t)auto_receive.aim_flag;
//		auto_receive.auto_position=data[i+14]&0x01;//大概分为在前哨（1）和基地（0）吧
		auto_receive.auto_revolve= 0;//(data[i+14]>>1)&0x01;//小陀螺
//		data_set=data[i+14];
		auto_receive.auto_upslope=data[i+7];//爬坡
//	  auto_receive.auto_downslope=(data[i+14]>>3)&0x01;
//	  sentry_mode_set =(data[i+14]>>4)&0x03;
	    sentry_mode_set = data[i+14];	
		last_yaw_ab_angle = gimbal_control.gimbal_yaw_motor.absolute_angle ;//左正右负
		last_pitch_ab_angle = gimbal_control.gimbal_pitch_motor.absolute_angle ;//上负下正
	                                                                   //yaw和pitch陀螺仪角度
		gimbal_control.gimbal_yaw_motor.absolute_angle_auto = gimbal_control.gimbal_yaw_motor.absolute_angle ;//左正右负
		gimbal_control.gimbal_pitch_motor.absolute_angle_auto = gimbal_control.gimbal_pitch_motor.absolute_angle ;//上负下正
			                                                               //y、p编码
		ecd_pitch = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
		ecd_yaw   = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
		
	   //--------------------------------MY-TEST
		 if(data[i+8]&0x80)//底盘vx_set
		{
			auto_chassis_x=-(float)(((data[i+8]&0x7f)*256+data[i+9]))/*/100.0f*0.01745329222222f*/;
		}
		else
		{
			auto_chassis_x=(float)(((data[i+8]&0x7f)*256+data[i+9]))/*/100.0f*0.01745329222222f*/;
		}


		if(data[i+10]&0x80)	//底盘vy_set
		{
			auto_chassis_y=-(float)(((data[i+10]&0x7f)*256+data[i+11]))/*/100.0f*0.01745329222222f*/;
		}		 
		else
		{
			auto_chassis_y=(float)(((data[i+10]&0x7f)*256+data[i+11]))/*/100.0f*0.01745329222222f*/;
		}

		if(data[i+12]&0x80)	//底盘wz_set
		{
			auto_chassis_wz=-(float)(((data[i+12]&0x7f)*256+data[i+13]))/*/100.0f*0.01745329222222f*/;
		}		 
		else
		{
			auto_chassis_wz=(float)(((data[i+12]&0x7f)*256+data[i+13]))/*/100.0f*0.01745329222222f*/;
		}

//		chassis_auto_status = data[i+14];

		if(Auto_data_t[0]&0x80)//add_pitch(PP)
		{
			Shoot_data[0]=(float)((Auto_data_t[0]&0x7f)*256+Auto_data_t[1])/*100.0f*/;
		}
		else
		{
			Shoot_data[0]=-(float)((Auto_data_t[0]&0x7f)*256+Auto_data_t[1])/*/100.0f*0.01745329222222f*/;
		}
		if(Auto_data_t[2]&0x80)	//add_yaw(YY)
		{
			Shoot_data[1]=-(float)((Auto_data_t[2]&0x7f)*256+Auto_data_t[3])/*/100.0f*0.01745329222222f*/;
		}
		else
		{
			Shoot_data[1]=(float)((Auto_data_t[2]&0x7f)*256+Auto_data_t[3])/*/100.0f*0.01745329222222f*/;
		}

	  
		
//		PP = Shoot_data[0]*180.0f/3.14f/100.0f;//*0.01745329222222f;//180.0f*3.14f;/;
//		YY = Shoot_data[1]*180.0f/3.14f/100.0f;//*0.01745329222222f;//180.0f*3.14f;//;
		 
		PP =Shoot_data[0]*0.001f;//*0.01745329222222f;//自瞄使用
		YY =Shoot_data[1]*0.001f;
		
		
		vision_p =Shoot_data[0];//射击使用
		vision_y =Shoot_data[1];
          if(get_robot_HP()<10)
						sentry_mode_set = 0;
			    if (auto_receive.aim_flag>0 && get_robot_HP() >= 10 && data[i+7]==0)//瞄到人
				  sentry_mode_set = 2;//切进攻
//					if (auto_receive.auto_upslope==1)//上坡
//				{
//					auto_receive.aim_flag=0;//不进行自瞄
//					sentry_mode_set = 0;//确保是移动姿态
////					auto_chassis_x = auto_chassis_x*1.15f;//速度放大
//				}	
//				  if (get_robot_HP() <= 150)//血量低，优先导航
//					sentry_mode_set = 0;
					if (sentry_mode_set !=0 && get_robot_HP() >= 10) //非移动姿态
					{
					if (sentry_mode_set == 2 && data[i+14]==0){
					auto_chassis_x = 0;
					auto_chassis_y = 0;}
			    auto_receive.auto_revolve=1;//切小陀螺
					}
		//视觉发过来的数据是放大一百倍的
		if(data[i+7])
		{
		 auto_receive.auto_revolve=0;			
		auto_chassis_x  = auto_chassis_x  * 0.015f;//底盘vx_set
		auto_chassis_y  = auto_chassis_y  * 0.009f;//底盘vy_set
		auto_chassis_wz = auto_chassis_wz * 0.01f;}//底盘wz_set
		else if(sentry_mode_set == 0 || auto_receive.Auto_Cap_U==3)
		{
    auto_receive.auto_revolve=0;			
		auto_chassis_x  = auto_chassis_x  * 0.02f;//底盘vx_set
		auto_chassis_y  = auto_chassis_y  * 0.02f;//底盘vy_set
		auto_chassis_wz = auto_chassis_wz * 0.01f;}//底盘wz_set
		else {
			auto_chassis_x  = auto_chassis_x  * 0.01f;//底盘vx_set
		auto_chassis_y  = auto_chassis_y  * 0.01f;//底盘vy_set
		auto_chassis_wz = auto_chassis_wz * 0.01f;
		}
		auto_vx_init = auto_chassis_x;
		
	//-----------------------------------------------------------------
		auto_receive.last_vision_flag = auto_receive.vision_flag;
}

void USART1_IRQHandler(void)
{
 
    int i=0;
    
  /* USER CODE BEGIN USART1_IRQn 0 */
	
	
if(__HAL_UART_GET_FLAG(&huart1 ,UART_FLAG_RXNE )!=RESET )
{ 


}
        
        
	
if(__HAL_UART_GET_FLAG(&huart1 ,UART_FLAG_IDLE )!=RESET )
{
	 USART1 ->SR;			//读SR 和 DR 寄存器 达到清除空闲中断的标志位
	 USART1->DR;
	 __HAL_UART_CLEAR_IDLEFLAG(&huart1 );
			for(;i<36;i++)
			{
				if(data[i]== 0xFF && data [i+15]== 0xFE)
				{
					 Aoto_data_work(i);//处理视觉信息
				}
			}
//            HAL_UART_DMAStop (&huart1 ); 	
			recv_end_flag = 1;//接收完成标志位
}

  /* USER CODE END USART1_IRQn 0 */
HAL_UART_IRQHandler(&huart1);

}


int P_flag = 0;
int32_t M_S[3] = {1,0,0};
char target[3] = {0x00,0x01,0x02};
int DF=0;

int32_t a[3] = {0,0,0};//占位，对烧饼无用
int32_t b[3] = {0,0,0};//占位，对烧饼无用

extern chassis_move_t chassis_move;
extern gimbal_control_t gimbal_control;
extern robot_interaction_data_t robot_interaction_data;
void MainFocus_task( void const * argument)
{
   __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//使能空闲中断
   __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);//使能接收中断
	
	auto_receive.last_vision_flag=0;
	auto_receive.vision_flag=0;
	
//	robot_interaction_data.data_cmd_id=0x0200;
//	robot_interaction_data.receiver_id=0x0107;
//	robot_interaction_data.sender_id=0x0109;
	
    while(1)
   {
		if( recv_end_flag == 1)  //接收完成标志
		{
			recv_end_flag = 0;//清除接收结束标志位
			memset(data,0,36);
		}
		//在OLED上打印add_yaw(pitch)、vx(y)_set数据
		auto_receive.auto_YY=YY;
		auto_receive.auto_PP=PP;
		auto_receive.auto_x= auto_chassis_x;
		auto_receive.auto_y= auto_chassis_y;
	
		get_send_data(&auto_manage.auto_send);//获取欧拉角、角速度、加速度、阵营数据
		send_data_resolve(&auto_manage.auto_send);//数据整合为hex数据包
		HAL_UART_Transmit_DMA(&huart1,auto_manage.auto_send.send_data ,49);//发送数据（DMA非阻塞式）
		HAL_UART_Receive_DMA(&huart1,data,36);//重新打开DMA接收
		osDelay(10);//100hz
    }
}
