/* 包含头文件 ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"

#include "sentry_console.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
AppType_e app_type;

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
void StartMusic(void);

/* 函数体 --------------------------------------------------------------------*/
void AppInit(void)
{
    BSP_Init();
	  //可根据需要加入配置（部分需提前在CUBE-MX内完成配置，详见BSP各.c文件说明）

    if (BSP_GPIO_ReadPin(&app_gpio))
    {
        app_type = GIMBAL_APP;
    }
    else
    {
        app_type = CHASSIS_APP;
    }

    if (app_type == GIMBAL_APP)
    {
        HAL_Delay(1000);
    }
    //识别管脚执行底盘和云台，执行对应的初始化任务
		
		

		
    SoftwareTimerTaskInit();
    IMU_TaskInit();
    ConsoleTaskInit();
    Comm_TaskInit();
    DetectTaskInit();

}

AppType_e GetAppType(void)
{
    return app_type;
}
