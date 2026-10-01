/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "detect_task.h"
#include "CAN_supercap_communication.h"
#include "chassis_power_control_with_supercap.h"
/*
    发：
        CAN1：pitch、yaw、底盘四个轮子电机
        CAN2：拨弹轮、左右摩擦轮、超级电容
    收：
        CAN1：pitch、yaw、底盘轮子
        CAN2：拨弹轮、左右摩擦轮、超级电容
*/

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define CHASSIS_CAN     hcan1
#define GIMBAL_CAN      hcan1
#define SHOOT_CAN       hcan2
#define SuperCap_CAN    hcan2


//----------超级电容获取------------------
#define get_SuperCap_measure(ptr, data) \
  {                                     \
    (ptr)->Ready = data[0];              \
    (ptr)->State = data[1];               \
    (ptr)->Energy = data[2];         \
    (ptr)->ChassisRealPower = data[3];        \
    (ptr)->BatVoltage = data[4];             \
  }

// 电机数据读取(更新)
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

/*
motor data,  0:chassis motor1 3508;1:chassis motor2 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; */
static motor_measure_t motor_chassis[6];

// 射击电机数据，0、1、2、3:四个摩擦轮 3508电机；4、5:拨弹电机 2006电机
 motor_measure_t motor_shoot[6];
  
CAN_TxHeaderTypeDef shoot_tx_message;
uint8_t shoot_can_send_data[8];
CAN_TxHeaderTypeDef trigger_tx_message;
uint8_t trigger_can_send_data[8];
 
static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    // 获取CAN返回数据
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan==&hcan1)
    {
        switch (rx_header.StdId)    // 底盘电机、云台yaw/pitch电机
        {
			
                case CAN_3508_M1_ID:
                case CAN_3508_M2_ID:
                case CAN_3508_M3_ID:
                case CAN_3508_M4_ID:
                case CAN_YAW_MOTOR_ID:
								case CAN_PIT_MOTOR_ID:
                {
                    static uint8_t i = 0;
                    // get motor id
                    i = rx_header.StdId - CAN_3508_M1_ID;
                    // 电机数据更新
                    get_motor_measure(&motor_chassis[i], rx_data);
                    detect_hook(CHASSIS_MOTOR1_TOE + i);
					detect_hook(YAW_GIMBAL_MOTOR_TOE);
					detect_hook(PITCH_GIMBAL_MOTOR_TOE);
                    break;
                }
				        case CAN_SuperCap_RX_ID:     // 超级电容CAN数据 0x100
                {
										transfer_SuperCap_measure(rx_data);
                    break;
                }
                default:break;
        }
    }
    if (hcan==&hcan2)     // 摩擦轮1/2、拨弹轮、超级电容
    {
        switch (rx_header.StdId)
        {

                case CAN_FRIC_1_MOTOR_ID:     // 摩擦轮1 0x201
                case CAN_FRIC_2_MOTOR_ID:     // 摩擦轮1 0x201
                case CAN_FRIC_3_MOTOR_ID:     // 摩擦轮1 0x201
                case CAN_FRIC_4_MOTOR_ID:     // 摩擦轮1 0x201
                {
					static uint8_t i = 0;
					i = rx_header.StdId - CAN_FRIC_1_MOTOR_ID;
                    get_motor_measure(&motor_shoot[i], rx_data);
					
					detect_hook(TRIGGER_MOTOR_1_TOE);
					detect_hook(TRIGGER_MOTOR_2_TOE);
					
					detect_hook(FRIC_MOTOR_1_TOE);
					detect_hook(FRIC_MOTOR_2_TOE);
					detect_hook(FRIC_MOTOR_3_TOE);
					detect_hook(FRIC_MOTOR_4_TOE);
             break;
                }
				
				case CAN_TRIGGER_MOTOR_ID1:{get_motor_measure(&motor_shoot[4], rx_data);break;}  // 拨弹轮 0x203
				case CAN_TRIGGER_MOTOR_ID2:{get_motor_measure(&motor_shoot[5], rx_data);break;}  // 拨弹轮 0x203
				
                default:break;
        }
    }
}

/**
 * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
 * @param[in]      rev1: (0x208) 保留，电机控制电流
 * @param[in]      rev2: (0x208) 保留，电机控制电流
 * @retval         none
 */


//void CAN_cmd_gimbal_p(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
//{
//    uint32_t send_mail_box;
//    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;    // 0x1FF
//    gimbal_tx_message.IDE = CAN_ID_STD;
//    gimbal_tx_message.RTR = CAN_RTR_DATA;
//    gimbal_tx_message.DLC = 0x08;
//    gimbal_can_send_data[0] = (yaw >> 8);
//    gimbal_can_send_data[1] = yaw;
//    gimbal_can_send_data[2] = (pitch >> 8);
//    gimbal_can_send_data[3] = pitch;
//    gimbal_can_send_data[4] = (rev1 >> 8);
//    gimbal_can_send_data[5] = rev1;
//    gimbal_can_send_data[6] = (rev2 >> 8);
//    gimbal_can_send_data[7] = rev2;
//    
//    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
//}


/**
 * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
//-------------------------------------can1--------------------------------
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;      // 0x200
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1FF;    // 0x1FF
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (rev1 >> 8);
    gimbal_can_send_data[5] = rev1;
    gimbal_can_send_data[6] = (rev2 >> 8);
    gimbal_can_send_data[7] = rev2;
    
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          发送发射机构电机控制电流
  * @param[in]      fric_motor_R: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      fric_motor_L: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      trigger_motor: (0x203) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor4: NULL（保留）
  * @retval         none
  */
//-------------------------------------can2--------------------------------
void CAN_cmd_fric(int16_t fric_motor_1, int16_t fric_motor_2, int16_t fric_motor_3, int16_t fric_motor_4)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = 0x200;      // 0x200
    shoot_tx_message.IDE = CAN_ID_STD;   //使用标准ID
	shoot_tx_message.RTR = CAN_RTR_DATA;   //发送数据帧而非远程帧
    shoot_tx_message.DLC = 0x08;         //数据长8字节1
    shoot_can_send_data[0] = fric_motor_1 >> 8;//高8位
    shoot_can_send_data[1] = fric_motor_1;     //低8位
    shoot_can_send_data[2] = fric_motor_2 >> 8;
    shoot_can_send_data[3] = fric_motor_2;
    shoot_can_send_data[4] = fric_motor_3 >> 8;
    shoot_can_send_data[5] = fric_motor_3;
    shoot_can_send_data[6] = fric_motor_4 >> 8;
    shoot_can_send_data[7] = fric_motor_4;
    
    HAL_CAN_AddTxMessage(&hcan2, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_cmd_trigger(int16_t trigger_motor_1, int16_t trigger_motor_2, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    trigger_tx_message.StdId = CAN_GIMBAL_ALL_ID;//CAN_SHOOT_ALL_ID;
    trigger_tx_message.IDE = CAN_ID_STD;
    trigger_tx_message.RTR = CAN_RTR_DATA;
    trigger_tx_message.DLC = 0x08;
    trigger_can_send_data[0] = trigger_motor_1 >> 8;
    trigger_can_send_data[1] = trigger_motor_1;
    trigger_can_send_data[2] = trigger_motor_2 >> 8;
    trigger_can_send_data[3] = trigger_motor_2;
    trigger_can_send_data[4] = rev1 >> 8;
    trigger_can_send_data[5] = rev1;
    trigger_can_send_data[6] = rev2 >> 8;
    trigger_can_send_data[7] = rev2;
    
    HAL_CAN_AddTxMessage(&hcan2, &trigger_tx_message, trigger_can_send_data, &send_mail_box);
}

/**
 * @brief          返回yaw 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
 * @brief          返回pitch 6020电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
 * @brief          返回拨弹电机 2006电机数据指针
 * @param[in]      none
 * @retval         电机数据指针
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_shoot[4];
//    return &motor_chassis[6];
}
const motor_measure_t *get_trigger2_motor_measure_point(void)
{
    return &motor_shoot[5];
//    return &motor_chassis[6];
}
/**
 * @brief          返回摩擦轮 3508、拨弹电机 2006电机数据指针
 * @param[in]      i: 电机编号,范围[0,2]    0、1——摩擦轮  2——拨弹轮
 * @retval         电机数据指针
 */
const motor_measure_t *get_shoot_motor_measure_point(uint8_t i)
{
        return &motor_shoot[i];
}

/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
