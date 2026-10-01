#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"
//Referee_Date *GetRefereeDataPoint(void);


typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;


//---------------------------------------2024.3.14
typedef __packed struct	//0x0001
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
 uint64_t SyncTimeStamp;
}game_status_t;

typedef __packed struct	//00002
{
 uint8_t winner;
}game_result_t;

typedef __packed struct	//0x0003有变
{   
uint16_t ally_1_robot_HP;   
uint16_t ally_2_robot_HP;   
uint16_t ally_3_robot_HP;   
uint16_t ally_4_robot_HP;   
uint16_t reserved;   
uint16_t ally_7_robot_HP;   
uint16_t ally_outpost_HP;   
uint16_t ally_base_HP; 
} game_robot_HP_t; 

typedef __packed struct	//0x0101
{
 uint32_t event_data;
}event_data_t;

typedef __packed struct	//0x0102
{
 uint8_t reserved;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct	//0x0104
{
 uint8_t level;
 uint8_t offending_robot_id;
 uint8_t count;
}referee_warning_t;

typedef __packed struct	//0x0105
{
 uint8_t dart_remaining_time;
 uint16_t dart_info;
}dart_info_t;

typedef __packed struct	//0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP; 
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef __packed struct   //0x0202  有变
{ 
  uint16_t reserved; 
  uint16_t reserved2; 
  float reserved3; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
}power_heat_data_t; 

typedef __packed struct	//0x0203
{
 float x;
 float y;
 float angle;
}robot_pos_t;

typedef __packed struct	//0x0204	机器人增益  有变
//{
// uint8_t recovery_buff;
// uint8_t cooling_buff;
// uint8_t defence_buff;
// uint8_t vulnerability_buff;
// uint16_t attack_buff;
//}buff_t;
{ 
   uint8_t recovery_buff;   
   uint16_t cooling_buff;   
   uint8_t defence_buff;   
   uint8_t vulnerability_buff; 
   uint16_t attack_buff; 
   uint8_t remaining_energy; 
}buff_t; 

typedef __packed struct	//0x0206
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;


typedef __packed struct	//0x0207
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

typedef __packed struct	//0x0208  有变
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
 uint16_t projectile_allowance_fortress; //新增
}projectile_allowance_t;

typedef __packed struct	//0x209  有变
{
 uint32_t rfid_status;
 uint8_t rfid_status_2; //新增
}rfid_status_t;

typedef __packed struct	//0x020A	当前飞镖发射站的状态
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;


typedef __packed struct	//0x020B  有变
{
 float hero_x;
 float hero_y;
 float engineer_x;
 float engineer_y;
 float standard_3_x;
 float standard_3_y;
 float standard_4_x;
 float standard_4_y;
 float reserved1;
 float reserved2;
}ground_robot_position_t;

typedef __packed struct	//0x020C  有变
{
uint16_t mark_progress;  
}radar_mark_data_t;

typedef __packed struct //0x20D   重要
{   
   uint32_t sentry_info; 
   uint16_t sentry_info_2; 
} sentry_info_t; 

typedef __packed struct	//0x020E
{
 uint8_t radar_info;
} radar_info_t;

typedef __packed struct	//0x0301
{
 uint16_t data_cmd_id;
 uint16_t sender_id;
 uint16_t receiver_id;
 uint8_t user_data[4];
}robot_interaction_data_t;

//云台手向机器人发送数据
typedef __packed struct//0x303
{
float target_position_x;
float target_position_y;
uint8_t cmd_keyboard;
uint8_t target_robot_id;
uint16_t cmd_source;
}map_command_t;

//typedef __packed struct	//0x0120	哨兵自主决策指令
//{
//	uint32_t sentry_cmd; 
//}sentry_cmd_t;

typedef __packed struct	//0x0121	雷达自主决策指令
{
uint8_t radar_cmd;
} radar_cmd_t;


extern uint8_t other_robot_live;

//----------------------------------------------------------
//UI设计
typedef __packed struct
{ 
uint8_t graphic_name[3]; 
uint32_t operate_tpye:3; 
uint32_t graphic_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11; 
}graphic_data_struct_t;

extern uint16_t get_17mm_allow(void);
extern uint8_t get_rfid_status(void);
extern uint16_t get_bullet_remaining(void);
extern uint16_t get_base_HP(void);
extern uint16_t get_outpost_HP(void);
extern uint16_t get_robot_HP(void);
extern uint16_t get_shooter_17mm_1_power_heat_data(void);
extern uint16_t get_shooter_17mm_2_power_heat_data(void);
extern void get_shooter_17mm_power_heat_data(uint16_t *heat);
extern void get_chassis_power_limit(uint16_t *chassis_power_limit_t);
extern void get_HP(uint16_t *hp_now, uint16_t *hp_max);
extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern uint8_t get_game_progress(void);
extern uint8_t get_robot_id(void);
extern uint8_t get_event_data(void);
extern void get_shoot_heat_limit(uint16_t *heat_limit);
extern void get_shoot_heat(uint16_t *heat);
//extern void get_shoot_heat2(uint16_t *heat2);
extern void Get_shooter_17mm_speed(float *shooter_17mm_speed_limit );
extern uint8_t get_robot_term(void);
extern uint8_t get_combat_status(void);
extern uint8_t get_sentry_mode(void);
extern uint8_t get_sentry_free_revive_confirm(void);


uint8_t get_other_robot_live(void);
//extern uint16_t get_other_outpost_HP(void);

uint16_t get_cooling_value(void);

extern uint8_t get_map_command_case(fp32* target_x, fp32* target_y);

extern uint8_t get_free_target_position(float* free_x, float* free_y);

fp32 get_shoot_speed(void);

#endif


