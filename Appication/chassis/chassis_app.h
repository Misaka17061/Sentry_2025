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
    fp32 wheel_perimeter; /* the perimeter(mm) of wheel *///wheel周长
    fp32 wheeltrack;      /* wheel track distance(mm) */  //轮距
    fp32 wheelbase;       /* wheelbase distance(mm) */    //轴距
    fp32 rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */  //相对于底盘中心的x轴旋转偏移（mm）
    fp32 rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */  //相对于底盘中心的y轴旋转偏移（mm）
} MechanicalStructure_t;

typedef struct
{
    Console_t*    					console;
    IMU_Data_t*   					imu;                      
    CAN_Object_t* 					chassis_can;              

    ChassisCtrlMode_e       ctrl_mode;     
	MechanicalStructure_t   structure;             //机械结构体
    M3508_Motor_t          	chassis_motor[4];
	M6020_Motor_t           chassis_steer_motor[4];               
	
	pid_t 					follow_pid;
	
    fp32 					vx;                      
    fp32 					vy;                    
    fp32 					vw;
	fp32 					wheel_rpm[4];
	
    fp32 					platform_yaw_ecd_angle;	
	fp32 					gimbal_yaw_ecd_angle;	
	fp32					spin_relative_angle;
		
	uint16_t 				super_flag;
	
		  float Super_Power_ratio;         //超电充电比
    float Chassis_super_power;
		uint8_t	SuperPower_State;
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
