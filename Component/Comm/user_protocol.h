#ifndef USER_PROTOCOL_H
#define USER_PROTOCOL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    RC_DATA_CMD_ID           = 0x0001,
    CHASSIS_INFO_CMD_ID      = 0x0002,
    GIMBAL_INFO_CMD_ID       = 0x0003,
} USER_CMD_ID_e;

#pragma pack(push,1)


#pragma pack(pop)
/* 宏定义 --------------------------------------------------------------------*/
#define USER_PROTOCOL_HEADER_SOF     0xAA
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);


#endif  // USER_PROTOCOL_H


