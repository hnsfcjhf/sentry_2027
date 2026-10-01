#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

//---------------------------------

//---------------------------------
frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_state;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t event_data;
ext_supply_projectile_action_t supply_projectile_action_t;
referee_warning_t referee_warning;


robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_pos;
buff_t bufff;
hurt_data_t hurt_data;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;


ground_robot_position_t ground_robot_position;
radar_mark_data_t radar_mark_data;
sentry_info_t sentry_info;
radar_info_t radar_info;
robot_interaction_data_t robot_interaction_data;
//sentry_cmd_t sentry_cmd;
radar_cmd_t radar_cmd;
//---------------------------
map_command_t map_command;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));


    memset(&event_data, 0, sizeof(event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));


    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&bufff, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
	memset(&rfid_status, 0, sizeof(rfid_status_t));


	memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
	memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
	memset(&sentry_info, 0, sizeof(sentry_info_t));
	memset(&radar_info, 0, sizeof(radar_info_t));
	memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
//  memset(&sentry_cmd, 0, sizeof(sentry_cmd_t));
	memset(&radar_cmd, 0, sizeof(radar_cmd_t));
	
	memset(&map_command, 0, sizeof(map_command_t));


}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&event_data, frame + index, sizeof(event_data_t));
        }
        break;
//        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
//        {
//            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
//        }
//        break;
//        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
//        {
//            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
//        }
//        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&bufff, frame + index, sizeof(buff_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
						memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
				}
				break;
				case RFID_STATUS_CMD_ID:
				{
						memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
				}
				break;
				case ROBOT_LOCATION_CMAD_ID:
				{
					  memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
				}
				break;
				case AOTO_DECISION_CMD_ID:
				{
					   memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
				}
				break;
				case MAP_COMMAND_CMD_ID:
				{
								memcpy(&map_command, frame + index, sizeof(map_command_t));			
				}
				break;
				    default:
						{
								break;
						}
		}
}
//---------------------------------------------------

//uint16_t get_17mm_allow(void)
//{
//	return projectile_allowance.projectile_allowance_17mm;
//}

uint32_t AA = 0x00080000;

uint8_t get_rfid_status(void)
{
//	rfid_status.rfid_status=AA;		//1111 1111 1111 1111 1111 1111 1111 1111
	return ((rfid_status.rfid_status >> 19) & 0x01);
}

void get_chassis_power_limit(uint16_t *chassis_power_limit_t)
{
    *chassis_power_limit_t =robot_status.chassis_power_limit;
}

uint16_t get_bullet_remaining(void)
{
	return projectile_allowance.projectile_allowance_42mm;
}
//------------------------------------基地血量
float Base_HP = 1500;
uint16_t get_base_HP(void)
{
//	return (uint16_t)Base_HP;
		return game_robot_HP.ally_base_HP;
}

//------------------------------------前哨站血量
uint16_t get_outpost_HP(void)
{
		return game_robot_HP.ally_outpost_HP;
}

//uint16_t get_other_outpost_HP(void)
//{

//	if((robot_status.robot_id)>0&&(robot_status.robot_id)<=7)
//	{
//		return game_robot_HP;
//	}
//	if((robot_status.robot_id)>100&&(robot_status.robot_id)<=107)
//	{
//		return game_robot_HP.red_outpost_HP;
//	}
//}

//------------------------------------哨兵血量
uint16_t get_robot_HP(void)
{	
	return robot_status.current_HP;
}

void get_shooter_17mm_power_heat_data(uint16_t *heat)
{
	*heat = power_heat_data.shooter_17mm_barrel_heat;
}

uint16_t get_chassis_power(void)
{
	return power_heat_data.reserved3;
}

void get_HP(uint16_t *hp_now, uint16_t *hp_max)
{
  *hp_now=robot_status.current_HP;
  *hp_max=robot_status.maximum_HP;
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data.reserved3;		//功率
    *buffer = power_heat_data.buffer_energy;	//缓冲能量
}

uint8_t get_game_progress(void)
{
   return game_state.game_progress;
}

uint8_t get_robot_id(void)
{
    return robot_status.robot_id;
}
//bit:30-31中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领

uint8_t get_event_data(void)
{
//	static uint32_t temp = 0x71111111;
//	return ((temp >> 29) & 0x03);
	return ((event_data.event_data >> 29) & 0x03);
}

void Get_shooter_17mm_speed(float *shooter_17mm_speed_limit )
{
	if(shoot_data.shooter_number==1)
	{
		*shooter_17mm_speed_limit=shoot_data.initial_speed;;
	}
	 
		
}

uint8_t get_combat_status(void)
{
    return sentry_info.sentry_info_2 & 0x01; // 1：脱战状态
}

uint8_t get_sentry_mode(void)
{
	return ((sentry_info.sentry_info_2 >> 12) & 0x03);//1为进攻2为防御3为移动
}

uint8_t get_sentry_free_revive_confirm(void)
{
	return ((sentry_info.sentry_info >> 19) & 0x01);//1为可免费复活
}

uint8_t get_robot_term()
{
	if(get_robot_id()>0&&get_robot_id()<=7)
		return 1;
	else if(get_robot_id()>100&&get_robot_id()<=107)
		return 2;
}

//uint8_t get_other_robot_live(void)
//{	
//	if(get_robot_term()==1)
//	{
//		if(game_robot_HP.blue_1_robot_HP>0) other_robot_live=1;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.blue_2_robot_HP>0) other_robot_live=2+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.blue_3_robot_HP>0) other_robot_live=4+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.blue_4_robot_HP>0) other_robot_live=8+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.blue_5_robot_HP>0) other_robot_live=16+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.blue_7_robot_HP>0) other_robot_live=32+other_robot_live;
//		else other_robot_live=0;
//	}
//	else if(get_robot_term()==2)
//	{
//		if(game_robot_HP.red_1_robot_HP>0) other_robot_live=1;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.red_2_robot_HP>0) other_robot_live=2+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.red_3_robot_HP>0) other_robot_live=4+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.red_4_robot_HP>0) other_robot_live=8+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.red_5_robot_HP>0) other_robot_live=16+other_robot_live;
//		else other_robot_live=0;
//		
//		if(game_robot_HP.red_7_robot_HP>0) other_robot_live=32+other_robot_live;
//		else other_robot_live=0;
//	}
//	else
//		other_robot_live=0;
//	
//	
//}


//------------------------------------------------------------------------------------------------------
fp32 get_shoot_speed()	//返回初速度
{
	return shoot_data.initial_speed;
}
void get_shoot_heat_limit(uint16_t *heat_limit)
{
	*heat_limit = robot_status.shooter_barrel_heat_limit;
}
void get_shoot_heat1(uint16_t *heat)
{
    *heat = power_heat_data.shooter_17mm_barrel_heat;
}

//----------------云台手发送数据-----------------------------
uint8_t get_map_command_case(float* target_x, float* target_y)
{
	
		*target_x=map_command.target_position_x; 
		*target_y=map_command.target_position_y;
		if(map_command.target_position_x==0 && map_command.target_position_y==0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	
}
//-------------------------------------------------
uint8_t get_free_target_position(float* free_x, float* free_y)
{

		*free_x =(float)( (robot_interaction_data.user_data[0]*256 + robot_interaction_data.user_data[1]) /100.0f);	
		*free_y =(float)( (robot_interaction_data.user_data[2]*256 + robot_interaction_data.user_data[3]) /100.0f);	

	if(*free_x==0 && *free_y==0)
	{
		return 0;
	}
	else
		return 1;
	
}

//-----------------------------------------------------
uint16_t get_cooling_value()
{
	return robot_status.shooter_barrel_cooling_value;
}
