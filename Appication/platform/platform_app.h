#ifndef PLATFORM_APP_H
#define PLATFORM_APP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "sentry_console.h"
#include "Motor/9025motor.h"
/* 类型定义 ------------------------------------------------------------------*/

typedef enum
{
    PLATFORM_RELAX = 0,
		PLATFORM_INIT,
		PLATFORM_FOLLOW_GIMBAL,
    COMPUTER_CONTROL,
    HUMAN_CONTROL,
} PlatformCtrlMode_e;

typedef struct
{
    Console_t*          console;
    IMU_Data_t*         imu;              
    CAN_Object_t*       platform_can;       
	
		fp32                gimbal_yaw_ecd_angle;
		pid_t								follow_pid;
	
    PlatformCtrlMode_e    ctrl_mode;       
    PlatformCtrlMode_e    last_ctrl_mode;  //

    Motor_9025_t       yaw_motor;
} PlatformHandle_t;
/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void PlatformAppConfig(void);

#endif  

