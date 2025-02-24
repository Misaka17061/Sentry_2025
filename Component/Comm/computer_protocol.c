/* 包含头文件 ----------------------------------------------------------------*/
#include "computer_protocol.h"
#include "detect_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
uint32_t max_diff_sof_time;
uint32_t diff_sof_time;
uint32_t now_sof_time;
uint32_t vision_offline;
/* 扩展变量 ------------------------------------------------------------------*/
//rx
Comm_VisionInfo_t vision_info;
Comm_NavInfo_t nav_info;

//tx
Comm_RobotInfo_t robot_info;
Comm_DecisionInfo_t decision_info;


/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
//上位机协议：Vision Protocol
void ComputerProtocol_ParseHandler(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    switch(cmd_id)
    {
        case VISION_DATA_CMD_ID:		//接收视觉自瞄数据
        {
			now_sof_time = BSP_GetTime_ms();
            vision_offline = CheckDeviceRunTime(OFFLINE_VISION_INFO);
            diff_sof_time = now_sof_time - vision_offline;
			if(diff_sof_time>max_diff_sof_time)
            {
              max_diff_sof_time = diff_sof_time;
            }
            memcpy(&vision_info, data, sizeof(Comm_VisionInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_VISION_INFO);      
        }break;
        case NAV_DATA_CMD_ID:			//接收视觉雷达数据
        {
            memcpy(&nav_info, data, sizeof(Comm_NavInfo_t));
            OfflineHandle_TimeUpdate(OFFLINE_NAV_INFO);
        }break;
    }
}

int TwoBytesToInt (uint8_t byte[2])
{
    unsigned int data = ((byte[1]<<8) | (byte[0]<<0));
    
    int result = *(int*)(&data);
    
    return result;
}

float FourBytesToFloat (uint8_t byte[4])
{
    unsigned int data = ((byte[3]<<24) | (byte[2]<<16) | (byte[1]<<8) | (byte[0]<<0));
    
    float current_mA = *(float*)(&data);
    
    return current_mA;
}

//rx
Comm_VisionInfo_t* VisionInfo_Pointer(void)
{
    return &vision_info;
}

Comm_NavInfo_t* NavInfo_Pointer(void)
{
    return &nav_info;
}

//tx
Comm_RobotInfo_t* RobotInfo_Pointer(void)
{
    return &robot_info;
}

Comm_DecisionInfo_t* DecisionInfo_Pointer(void)
{
		return &decision_info;
}	

