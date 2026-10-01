#ifndef REFEREE_DISPATCHTASK_H
#define REFEREE_DISPATCHTASK_H

#include "crc8_crc16.h"   // 如果已有此头文件可保留，否则内部定义
#include "referee.h"

/* ==================== 帧格式定义 ==================== */
//typedef __packed struct
//{
//    uint8_t  sof;          // 帧起始字节，固定 0xA5
//    uint16_t data_length;  // 数据段长度
//    uint8_t  seq;          // 包序号
//    uint8_t  crc8;         // 帧头 CRC8 校验
//} frame_header_t;

/* ==================== 哨兵自主决策指令结构体（不含帧尾） ==================== */
// 0x0301 交互数据帧结构（不含帧尾CRC16）
typedef __packed struct {
    frame_header_t head;          // 5字节帧头
    uint16_t cmd_id;              // 0x0301
    // 数据段（长度10）
    uint16_t sub_cmd_id;          // 0x0120
    uint16_t sender_id;           // 哨兵机器人ID（红7 / 蓝107）
    uint16_t receiver_id;         // 0x8080（服务器）
    uint8_t  data[4];             // 4字节姿态数据
} sentry_interaction_frame_t;

/* ==================== 姿态相关宏定义（符合协议位定义） ==================== */
#define SENTRY_POSTURE_BIT_OFFSET   21   // 姿态值位于 bit21~22
#define SENTRY_POSTURE_MASK         (0x3 << SENTRY_POSTURE_BIT_OFFSET)

#define SENTRY_POSTURE_ATTACK       1    // 进攻姿态
#define SENTRY_POSTURE_DEFENSE      2    // 防御姿态
#define SENTRY_POSTURE_MOVE         3    // 移动姿态

/* ==================== CRC 常量与函数声明 ==================== */
extern const unsigned char CRC8_INIT;
extern uint16_t CRC_INIT;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

/* ==================== 外部变量与函数声明 ==================== */
//extern sentry_cmd_t sentry_cmd;
extern uint8_t sentry_mode_set;    // 外部定义的模式选择变量（0=移动,1=防御,2=进攻）

void Send_ClientData_Task(void const *pvParameters);

#endif /* REFEREE_DISPATCHTASK_H */
