#ifndef	CHASSIS_APP_H
#define	CHASSIS_APP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "sentry_console.h"
#include "computer_protocol.h"
#include "decision_task.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum       //模式
{
    CHASSIS_RELAX = 0,
		CHASSIS_STOP,	
    CHASSIS_FOLLOW_GIMBAL,      
    CHASSIS_SEPARATE_GIMBAL,    
    CHASSIS_SPIN,      
		CHASSIS_SENTRY,
} ChassisCtrlMode_e;


typedef struct
{
    Console_t*    console;
    IMU_Data_t*   imu;                      
    CAN_Object_t* chassis_can;              

    ChassisCtrlMode_e       ctrl_mode;                         
    M3508_Motor_t          	chassis_motor[4];
		M6020_Motor_t           chassis_steer_motor[4];               
	
    fp32 vx;                      
    fp32 vy;                    
    fp32 vw;
    fp32 platform_yaw_ecd_angle;	
		fp32 gimbal_yaw_ecd_angle;	
		pid_t follow_pid;
    fp32 wheel_rpm[4];

		uint16_t turnFlag[4];
		fp32 lastSteeringAngletarget[4];
		fp32 steeringAngleTarget[4]; 
		fp32 steeringAngle[4];                     
		fp32 last_steeringAngle[4];
	 
} ChassisHandle_t;

typedef struct
{
    uint8_t head;
    fp32  	x;
    fp32  	y;
		fp32    w;
		uint8_t data_tail; 
} RadarDatabase_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Chassis_AppConfig(void);

#endif
