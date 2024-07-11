/* 包含头文件 ----------------------------------------------------------------*/
#include "user_protocol.h"
#include "string.h"
#include "RemoteControl/remote_control.h"
#include "detect_task.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

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
            RC_DataParser(RC_GetDataPointer(), data, len);
            OfflineHandle_TimeUpdate(OFFLINE_DBUS);
        }break;

    }
}



