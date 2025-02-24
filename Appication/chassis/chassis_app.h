#ifndef	CHASSIS_APP_H
#define	CHASSIS_APP_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "sentry_console.h"
#include "computer_protocol.h"
#include "decision_task.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef enum       //ģʽ
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
    fp32 wheel_perimeter; /* the perimeter(mm) of wheel *///wheel�ܳ�
    fp32 wheeltrack;      /* wheel track distance(mm) */  //�־�
    fp32 wheelbase;       /* wheelbase distance(mm) */    //���
    fp32 rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */  //����ڵ������ĵ�x����תƫ�ƣ�mm��
    fp32 rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */  //����ڵ������ĵ�y����תƫ�ƣ�mm��
} MechanicalStructure_t;

typedef struct
{
    Console_t*    					console;
    IMU_Data_t*   					imu;                      
    CAN_Object_t* 					chassis_can;              

    ChassisCtrlMode_e       ctrl_mode;     
	MechanicalStructure_t   structure;             //��е�ṹ��
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
	
		  float Super_Power_ratio;         //�������
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

/* �궨�� --------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void Chassis_AppConfig(void);

#endif
