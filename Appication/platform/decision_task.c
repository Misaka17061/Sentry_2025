/*******************************************************************************
 * 
 * File name: 
 * Author:         Version:        Date: 
 * Description: 
 * Function List:
 *   1. 
 * History:
 *      <author> <time>  <version > <desc>
*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "decision_task.h"
#include "cmsis_os.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId DecisionTaskHandle;

/* 扩展变量 ------------------------------------------------------------------*/
NavDecision_e Decision_handle;

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void DecisionTask(void const*argument)
{
		for(;;)
	  {
		
		}
}

void DecisionTaskInit(void)
{
    osThreadDef(decision_task, DecisionTask, osPriorityNormal, 0, 256);
    DecisionTaskHandle = osThreadCreate(osThread(decision_task), NULL);
}



