/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-08-27 15:06:50
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-11-18 21:27:26
 * @FilePath: \MDK-ARMd:\RM_work\Hero_GroupWork\Hero2025_V1.3\Components\drvices\Super_Capacitor\super_capacitor.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
/* 包含头文件 ----------------------------------------------------------------*/
#include "super_capacitor.h"
#include "sentry_console.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

uint16_t total_current_g;
static uint16_t super_power_state = 0;

Cap_power cap_info;
/* 扩展变量 ------------------------------------------------------------------*/
extern ChassisHandle_t chassis_handle;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/




/*************************************************/
void CAP_PowerParser(Cap_power *received_data, uint8_t *can_receive_data, uint16_t len)
{
            uint16_t in_v_received = (can_receive_data[1] << 8) | can_receive_data[0];
            uint16_t cap_v_received = (can_receive_data[3] << 8) | can_receive_data[2];
            uint16_t in_c_received = (can_receive_data[5] << 8) | can_receive_data[4];
            uint8_t cap_percent_received = can_receive_data[6];
            uint8_t power_received = can_receive_data[7];

            float in_v = (float)in_v_received / 100.0f;
            float cap_v = (float)cap_v_received / 100.0f;
            float in_c = (float)in_c_received / 100.0f;
            float power_set = (float)power_received;
						float in_p  = in_v * in_c;
					
            received_data->in_v         = in_v;
            received_data->cap_v        = cap_v;
            received_data->in_c         = in_c;
            received_data->cap_percent  = cap_percent_received;
					  received_data->in_power		 = in_p;
	
						RefereeSystem_PowerHeatData_Pointer()->chassis_power = received_data->in_power;

}

Cap_power* CAP_GetDataPointer(void) //传出一个*P指针访问值。
{
    return &cap_info;             //具体内容
}
/* 
   功能: 通过CAN总线发送超级电容的消息。
   参数: obj - 指向CAN对象的指针，包含要发送的消息。
   返回值: 无。
*/
void SuperCap_SendMessage(CAN_Object_t* obj)

{
    static uint16_t chassis_power_limit=0;
    static uint16_t chassis_power=0;
    static uint16_t chassis_buff=0;
    static uint8_t TxData[8] = {0};
    
    chassis_power_limit = RefereeSystem_RobotState_Pointer()->chassis_power_limit;
    for(uint8_t i = 0; i < 4; i++)
        {
            // total_current_g += fabs(ChassisMotor_Pointer(i)->given_current);
        }
        if(chassis_handle.Super_Power_ratio<30)  //限制超电过低
        {
          console.SuperPower_cmd=Power_IN;  
        }
    chassis_power = RefereeSystem_PowerHeatData_Pointer()->chassis_power;
    chassis_buff = RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
    
    TxData[0] = chassis_power_limit>>8;
    TxData[1] = chassis_power_limit;
    TxData[2] = console.SuperPower_cmd;
    TxData[3] = chassis_buff>>8;
    TxData[4] = chassis_buff;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;
    BSP_CAN_TransmitData(obj, 0x100, TxData, 8);
}



//超电电机转速增益,没用
// void Chassis_CapPower(ChassisHandle_t* chassis_handle)
// {
//     fp32 total_current_limit = 0.0f;
//     fp32 total_current = 0.0f;
//     uint8_t robot_id = RefereeSystem_GetRobotID();

//     if (robot_id == 0 || CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
//     {
//         total_current_limit = 8000;
//     }
//     else if (robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER)
//     {
//         total_current_limit = MAX_TOTAL_CURRENT_LIMIT;
//     }
//     else
//     {
//         if(chassis_handle->Super_Power_ratio < 18)
//         {
//             fp32 power_scale;
//             if(chassis_handle->Super_Power_ratio > 16.0f)
//             {
//                 //scale down WARNING_POWER_BUFF
//                 power_scale = (chassis_handle->Super_Power_ratio-14) / 10;
//             }
//             else
//             {
//                 //only left 10% of WARNING_POWER_BUFF
//                 power_scale = 1.0f / 10;
//             }
//             //scale down
//             total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
//         }
//         else
//         {
//             //power > WARNING_POWER
//             if(chassis_handle->Super_Power_ratio <20 )
//             {
//                 fp32 power_scale;
//                 //power < 80w
//                 if(chassis_handle->Super_Power_ratio > 19)
//                 {
//                     //scale down
//                     power_scale = (chassis_handle->Super_Power_ratio-19) / 1;

//                 }
//                 //power > 80w
//                 else
//                 {
//                     power_scale = 0.0f;
//                 }

//                 total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
//             }
//             //power < WARNING_POWER
//             else
//             {
//                 total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
//             }
//         }
//     }

//     //calculate the original motor current set
//     for(uint8_t i = 0; i < 4; i++)
//     {
//         total_current += fabs(chassis_handle->chassis_motor[i].current_set);
//     }


//     if(total_current > total_current_limit)
//     {
//         fp32 current_scale = total_current_limit / total_current;
//         chassis_handle->chassis_motor[0].current_set *= current_scale;
//         chassis_handle->chassis_motor[1].current_set *= current_scale;
//         chassis_handle->chassis_motor[2].current_set *= current_scale;
//         chassis_handle->chassis_motor[3].current_set *= current_scale;
//     }
// }

