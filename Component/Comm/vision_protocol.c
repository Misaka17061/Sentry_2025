/* 包含头文件 ----------------------------------------------------------------*/
#include "vision_protocol.h"
#include "detect_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
Comm_VisionInfo_t vision_info;
Comm_RadarInfo_t radar_info;
Comm_RobotInfo_t robot_info;

uint32_t max_diff_sof_time;
uint32_t diff_sof_time;
uint32_t now_sof_time;
uint32_t vision_offline;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
//视觉协议：Vision Protocol
void VisionProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    switch(cmd_id)
    {
        case VISION_DATA_CMD_ID:
        {
            
            now_sof_time = BSP_GetTime_ms();
            vision_offline = CheckDeviceRunTime(OFFLINE_VISION_INFO);
            diff_sof_time = now_sof_time - vision_offline;
            memcpy(&vision_info, data, sizeof(Comm_VisionInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_VISION_INFO);
            
            if(diff_sof_time>max_diff_sof_time)
            {
              max_diff_sof_time = diff_sof_time;
            }
        }break;

        case RADAR_DATA_CMD_ID:
        {
            memcpy(&radar_info, data, sizeof(Comm_RadarInfo_t));
        }break;
    }
}

Comm_VisionInfo_t* VisionInfo_Pointer(void)
{

    return &vision_info;
}

Comm_RadarInfo_t* RadarInfo_Pointer(void)
{
    return &radar_info;
}

Comm_RobotInfo_t* RobotInfo_Pointer(void)
{
		return &robot_info;
}



