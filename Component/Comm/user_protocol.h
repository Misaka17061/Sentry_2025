#ifndef USER_PROTOCOL_H
#define USER_PROTOCOL_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "comm_protocol.h"
#include "gimbal_app.h"
#include "chassis_app.h"
#include "platform_app.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    RC_DATA_CMD_ID           			= 0x0001,
    CHASSIS_INFO_CMD_ID      			= 0x0002,
    GIMBAL_INFO_CMD_ID       			= 0x0003,
		PLATFORM_INFO_CMD_ID					= 0X0004,
		NAV_CMD_ID                   	= 0x0005,
} USER_CMD_ID_e;

typedef struct
{
    ChassisCtrlMode_e mode;
} Comm_ChassisInfo_t;

typedef struct
{
    GimbalCtrlMode_e mode;
		fp32 yaw_ecd_angle;
} Comm_GimbalInfo_t;

typedef struct
{
		PlatformCtrlMode_e mode;
		fp32 yaw_ecd_angle;
		fp32 gyro_angle;
} Comm_PlatformInfo_t;

typedef struct
{
		float vx;
		float vy;
		float vw;
}	Comm_NavCmd_t;

/* 宏定义 --------------------------------------------------------------------*/
#define USER_PROTOCOL_HEADER_SOF     0xAA
#define GIMBAL_CHASSIS_DATA_FIFO_SIZE   (1024u)
/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len);
Comm_ChassisInfo_t* ChassisInfo_Pointer(void);
Comm_GimbalInfo_t* GimbalInfo_Pointer(void);
Comm_PlatformInfo_t* PlatformInfo_Pointer(void);
Comm_NavCmd_t* NavCmd_Pointer(void);
#endif  // USER_PROTOCOL_H


