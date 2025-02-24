#ifndef	GIMBAL_APP_H
#define	GIMBAL_APP_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "IMU/imu_driver.h"
#include "Motor/motor.h"
#include "Motor/9025motor.h"
#include "sentry_console.h"
#include "computer_protocol.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    GIMBAL_RELAX = 0,       
    GIMBAL_INIT,
	GIMBAL_FOLLOW_PLATFORM,
    GIMBAL_NORMAL,
    GIMBAL_VISION_AIM,
	GIMBAL_SENTRY,
} GimbalCtrlMode_e;

typedef struct
{
    fp32            relative_angle; /* unit: degree */
    fp32            gyro_angle;
    fp32            palstance;      /* uint: degree/s */
} GimbalSensor_t;

typedef struct
{
    Console_t*          console;
    IMU_Data_t*         imu;              
    CAN_Object_t*       gimbal_can;       
	
    GimbalCtrlMode_e    ctrl_mode;       
    GimbalCtrlMode_e    last_ctrl_mode;  
    
	fp32 				platform_yaw_ecd_angle;	
	fp32 				platform_yaw_gyro_angle;
	
	pid_t				follow_pid;
    pid_t               gimbal_vision_yaw_pid;
    pid_t               gimbal_vision_pitch_pid;

    M6020_Motor_t       yaw_motor;
    M6020_Motor_t       pitch_motor;
} GimbalHandle_t;

typedef enum
{
    AIM_NO = 0,
    AIM_RIGHT,
} VisionAim_e;

typedef enum
{
    RELAX = 0,
    FOLLOW,
	FIRST_AIMING,
	COMPLETE_AIMING,
} AimContorl_e;

typedef struct
{
    float    pitch;
	float    yaw;
	float 	 palstance;
	float    accelerated_speed;
    float    last_pitch;
    float    last_yaw;  
	float	 last_yaw_form_center;
	float 	 last_palstance;
	float    last_accelerated_speed;
	int    	 can_shoot;
	int    	 move_state;
	int    	 last_move_state;
   	int 	 state;     //VisionState_e  0x01->Comm_Successed
    VisionAim_e 	yaw_success;
    VisionAim_e 	pitch_success;
} VisionDatabase_t;

typedef struct
{
AimContorl_e aim_mode;
uint32_t pid_SAtime;
uint32_t aiming_time;
uint32_t stay_time;
uint32_t systeam_time;
fp32 tol_angle ;  										//人为规定消抖角度范围(-tol_angle,tol_angle)
uint32_t tol_time ;										//the aiming-done continueous time that user set 人为规定
uint32_t Sstart_time,Astart_time;			//消抖持续时间,单次自瞄持续时间
int first_aim ;												//初入消抖范围标志
int aim_flag ; 												//新的单次自瞄标志
float Ap_parm;
float Sp_parm;
int enable_paramSA;
}AutoAim_t;
/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void GimbalAppConfig(void);

#endif
