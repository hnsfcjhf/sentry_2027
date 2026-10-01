/**
  ****************************(C) COPYRIGHT 2025 群星战队****************************
  * @file       CAN_supercap_communication.c/h
  * @brief      这里是群星战队超级电容数据定义、发送、接收与解析的库
  * @note       
  * @history
  *   版本          日期           作者                     变更
  *   V1.0.2516     2025-4-20     魔法电容使              1. 完成
	* 	V1.1.2517	    2025-4-25     魔法电容使				      2. 添加功率计算，新增功率补偿
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 群星战队****************************
*/


#include "CAN_supercap_communication.h"
#include "main.h"
#include "can.h"
#include "referee.h"

#define SUPERCAP_CAN    hcan1

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_SuperCapRXDataTypeDef CAN_SuperCapRXData = {0};

static CAN_TxHeaderTypeDef can_tx_message;

uint8_t PowerOffset =0;


uint32_t LastCapTick = 0; //上一次收到超电信号的时间戳
uint32_t NowCapTick = 0;  //现在收到超电信号的时间戳

uint32_t chassis_energy_in_gamming =0;


/**
  * @brief          获取超级电容在线状态，1在线，0离线
  * @retval         none
  */
uint8_t get_supercap_online_state(void){

  NowCapTick = HAL_GetTick(); //更新时间戳

  uint32_t DeltaCapTick = 0;
  DeltaCapTick = NowCapTick - LastCapTick; //计算时间差

	if(get_game_progress() == 4){
		//比赛开始的时候，开始统计消耗的能量

		chassis_energy_in_gamming += CAN_SuperCapRXData.BatPower * DeltaCapTick *0.001f;
    // 因为STM32的系统定时器是1ms的周期，所以*0.001，单位化为秒（S），能量单位才是焦耳（J）
	}

  if(DeltaCapTick > 1000){
    //如果时间差大于1s，说明超电信号丢失，返回超电离线的标志
    return 0;
  }else {
    //如果时间差小于1s，说明超电信号正常，返回超电在线的标志
    return 1;
  }
}

/**
  * @brief          获取根据超级电容功率统计的底盘消耗能量，单位：焦耳（J）
  * @retval         none
  */
uint32_t get_chassis_energy_from_supercap(void){

  return chassis_energy_in_gamming;
}


//----------超级电容数据从CAN传到结构体------------------
void transfer_SuperCap_measure(uint8_t *data)
{
  LastCapTick = HAL_GetTick();
  CAN_SuperCapRXData.SuperCapReady = (SuperCapReadyEnum)data[0];
  CAN_SuperCapRXData.SuperCapState = (SuperCapStateEnum)data[1];
  CAN_SuperCapRXData.SuperCapEnergy = data[2];
  CAN_SuperCapRXData.ChassisPower = data[3] << 1; //左移一位是为了扩大范围，超电发出来的的时候右移了一位
  CAN_SuperCapRXData.BatVoltage = data[4];
  CAN_SuperCapRXData.BatPower = data[5];
}



/**
  * @brief          获取超级电容的运行状态，具体查看枚举SuperCapStateEnum
  * @retval         none
  */
SuperCapStateEnum get_supercap_state(void){
  return (SuperCapStateEnum)CAN_SuperCapRXData.SuperCapState;
}

/**
  * @brief          获取超级电容读到的电池电压，单位V
  * @retval         none
  */
float get_battery_voltage_from_supercap(void){
  return (float)CAN_SuperCapRXData.BatVoltage * 0.1f;
}

/**
  * @brief          获取超级电容可用能量，范围：0-100%
  * @retval         none
  */
uint8_t get_supercap_energy(void){
  return CAN_SuperCapRXData.SuperCapEnergy;
}

/**
  * @brief          设置超级电容功率补偿
  * @retval         none
  */
void set_supercap_power_offset(uint8_t offset){
  PowerOffset = offset;
}

/**
  * @brief          发送超级电容数据
  * @param[in]      Enable: 超级电容使能
  * @param[in]      Charge: 超级电容充电，在PLUS版本中无效
  * @param[in]      PowerLimit: 裁判系统功率限制
  * @param[in]      Chargepower: 超级电容充电功率，在PLUS版本中无效
  * @retval         none
  */
void CAN_cmd_SuperCap(CAN_SuperCapTXDataTypeDef * TX_Temp)
{
    uint8_t CAN_tx_data[8];
    uint32_t send_mail_box;
    can_tx_message.StdId = SUPERCAP_TX_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    CAN_tx_data[0] = TX_Temp-> Enable;
    CAN_tx_data[1] = TX_Temp-> Charge;
    CAN_tx_data[2] = TX_Temp-> Powerlimit - PowerOffset;
    CAN_tx_data[3] = TX_Temp-> ChargePower;

    HAL_CAN_AddTxMessage(&SUPERCAP_CAN, &can_tx_message, CAN_tx_data, &send_mail_box);
}

