/* 包含头文件 ----------------------------------------------------------------*/
#include "user_protocol.h"
#include "string.h"
#include "RemoteControl/remote_control.h"
#include "detect_task.h"
#include "RemoteControl/et16s_rf209.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
Comm_ChassisInfo_t 	chassis_info;
Comm_GimbalInfo_t 	gimbal_info;
Comm_PlatformInfo_t platform_info;
Comm_NavCmd_t 			nav_cmd;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: UserProtocol_ParseHandler
 * Description: 用户自定义数据解析处理
 * Input: cmd_id 协议命令码
 *        data 数据指针
 *        len 数据长度
 * Return: 无
*************************************************/
void UserProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    switch(cmd_id)
    {
        case RC_DATA_CMD_ID:
        {
//            RC_gimbal_DataParser(WBUS_GetDataPointer(), data, len);
//						if(WBUS_GetDataPointer()->Flag == 0||WBUS_GetDataPointer()->Flag== 4)
//								OfflineHandle_TimeUpdate(OFFLINE_WBUS);
			RC_DataParser(RC_GetDataPointer(), data, len);
			OfflineHandle_TimeUpdate(OFFLINE_DBUS);
        }break;
				
		case CHASSIS_INFO_CMD_ID:
        {
            memcpy(&chassis_info,  data, sizeof(Comm_ChassisInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_CHASSIS_INFO);
        }break;

        case GIMBAL_INFO_CMD_ID:
        {
            memcpy(&gimbal_info,  data, sizeof(Comm_GimbalInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_INFO);
        }break;
				
		case PLATFORM_INFO_CMD_ID:
		{
            memcpy(&platform_info,  data, sizeof(Comm_PlatformInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_PLATFORM_INFO);
		}break;
				
		case NAV_CMD_ID:
		{
            memcpy(&nav_cmd,  data, sizeof(Comm_NavCmd_t));
            OfflineHandle_TimeUpdate(OFFLINE_NAV_INFO);
		}    break;
		case CAP_INFO_CMD_ID:
        {
			CAP_PowerParser(CAP_GetDataPointer(), data, len);
            OfflineHandle_TimeUpdate(OFFLINE_CAP_INFO);
        }break;
    }
}

/*************************************************
 * Function: ChassisInfo_Pointer
 * Description: 获取底盘信息数据
 * Input: 无
 * Return: 底盘信息数据指针
*************************************************/
Comm_ChassisInfo_t* ChassisInfo_Pointer(void)
{
    return &chassis_info;
}

/*************************************************
 * Function: GimbalInfo_Pointer
 * Description: 获取云台信息数据
 * Input: 无
 * Return: 云台信息数据指针
*************************************************/
Comm_GimbalInfo_t* GimbalInfo_Pointer(void)
{
    return &gimbal_info;
}

Comm_PlatformInfo_t* PlatformInfo_Pointer(void)
{
		return &platform_info;
}

Comm_NavCmd_t* NavCmd_Pointer(void)
{
		return &nav_cmd;
}  



