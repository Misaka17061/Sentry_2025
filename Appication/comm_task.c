/* 包含头文件 ----------------------------------------------------------------*/
#include "comm_task.h"
#include "cmsis_os.h"
#include "sentry_def.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId CommTaskHandle;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void Comm_Task(void const*argument)
{
    for(;;)
    {
        Comm_ReceiveDataHandler();//处理链表内各接收结构体缓冲区的数据
        Comm_TransmitDataHandler();
        osDelay(COMM_TASK_PERIOD);
    }
}

void Comm_TaskInit(void)
{
    osThreadDef(comm_task, Comm_Task, osPriorityNormal, 0, 256);
    CommTaskHandle = osThreadCreate(osThread(comm_task), NULL);
}
