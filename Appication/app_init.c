/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "app_init.h"
#include "Buzzer/buzzer.h"
#include "imu_task.h"
#include "comm_task.h"
#include "timer_task.h"
#include "detect_task.h"

#include "sentry_console.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/
AppType_e app_type;

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/
void StartMusic(void);

/* ������ --------------------------------------------------------------------*/
void AppInit(void)
{
    BSP_Init();
	  //�ɸ�����Ҫ�������ã���������ǰ��CUBE-MX��������ã����BSP��.c�ļ�˵����

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
    //ʶ��ܽ�ִ�е��̺���̨��ִ�ж�Ӧ�ĳ�ʼ������
		
		

		
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
