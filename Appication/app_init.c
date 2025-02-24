/* 包含头文件 ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"

#include "chassis_app.h"
#include "chassis_task.h"
#include "gimbal_app.h"
#include "gimbal_task.h"
#include "platform_app.h"
#include "platform_task.h"
#include "shoot_task.h"

#include "sentry_console.h"
#include "decision_task.h"

/*私有类型定义 --------------------------------------------------------------*/

/*私有宏定义 ----------------------------------------------------------------*/

/*私有变量 ------------------------------------------------------------------*/
AppType_e app_type;

/*扩展变量------------------------------------------------------------------*/

/*私有函数原形 -------------------------------------------------------------*/
void StartMusic(void);

/*函数体 --------------------------------------------------------------------*/  
void AppInit(void)
{
    BSP_Init();

    if (BSP_GPIO_ReadPin(&app_gpio))
    {
        app_type = GIMBAL_APP;
    }
    else
    {
        app_type = CHASSIS_APP;
    }
		
		if (BSP_GPIO_ReadPin(&app_gpio2))
		{
				app_type = RADAR_APP;
		}

    if (app_type == GIMBAL_APP)
    {
        HAL_Delay(1000);
		}
		else if (app_type == RADAR_APP)
    {
        HAL_Delay(500);
		}
		
    SoftwareTimerTaskInit();
    IMU_TaskInit();
    Comm_TaskInit();
    DetectTaskInit();
		ConsoleTaskInit();
		
		if(app_type == GIMBAL_APP)
    {
        GimbalAppConfig();
        ShootTaskInit();
        GimbalTaskInit();  
		//StartMusic();
    }
		else if(app_type == RADAR_APP)
		{
				PlatformAppConfig();
				PlatformTaskInit();
		}
		else if(app_type == CHASSIS_APP)
		{
				Chassis_AppConfig();
				ChassisTaskInit();
		}

}


void StartMusic()
{

}  
