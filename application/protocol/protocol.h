#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,//比赛状态数据
    GAME_RESULT_CMD_ID                = 0x0002,//比赛结果数据
    GAME_ROBOT_HP_CMD_ID              = 0x0003,//机器人血量数据
    FIELD_EVENTS_CMD_ID               = 0x0101,//场地事件数据
//    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,//场地补给站数据标识
//    SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103,//裁判系统没有
    REFEREE_WARNING_CMD_ID            = 0x0104,//裁判警告信息
    ROBOT_STATE_CMD_ID                = 0x0201,//机器人性能体系数据
    POWER_HEAT_DATA_CMD_ID            = 0x0202,//实时缓冲能量、热量数据
    ROBOT_POS_CMD_ID                  = 0x0203,//机器人位置数据
    BUFF_MUSK_CMD_ID                  = 0x0204,//机器人增益、底盘能量数据
    ROBOT_HURT_CMD_ID                 = 0x0206,//伤害状态数据
    SHOOT_DATA_CMD_ID                 = 0x0207,//实时射击数据
    BULLET_REMAINING_CMD_ID           = 0x0208,// 子弹剩余发射数
		RFID_STATUS_CMD_ID                = 0x0209,//机器人 RFID 状态
    ROBOT_LOCATION_CMAD_ID            = 0x020B,//地面机器人位置数据
	  AOTO_DECISION_CMD_ID              = 0x020D,//哨兵自主决策信息
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,//机器人间交互数据
	
   	MAP_COMMAND_CMD_ID				  =0X0303,//接收云台手向机器人发送数据
    IDCustomData,
}referee_cmd_id_t;
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
