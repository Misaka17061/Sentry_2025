#ifndef	SHOOT_TASK_H
#define SHOOT_TASK_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "RemoteControl/remote_control.h"
#include "Motor/motor.h"
#include "Motor/blocked.h"
#include "ADRC.h"
#include "pid.h"
#include "sentry_console.h"
#include "user_protocol.h"

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    SHOOT_RELAX = 0,         
    SHOOT_START,
    SHOOT_STOP,
} ShootCtrlMode_e;

typedef enum
{
    TRIGGER_END = 0,
    TRIGGER_BEGIN,
    TRIGGERING,
		RAPID_TRIGGERING,
		TRIGGER_BACKING,
} TriggerState_e;

typedef struct
{
		uint32_t start_time;
		uint32_t threshold;
		uint32_t backoff_time;
		fp32 backoff_angle;
}Trigger_back_t;


typedef struct
{
    Console_t*      console;
    CAN_Object_t*   shoot_can;    //
	
    ShootCtrlMode_e 	ctrl_mode;
    M2006_Motor_t    	trigger_motor;
    M3508_Motor_t  		fric_wheel_motor[2];
  
    
    TriggerState_e  trigger_state;
		Trigger_back_t 	trigger_back;
		float shooter_speedfeedback;

    uint16_t        shooter_heat_cooling_rate;
    uint16_t        shooter_heat_cooling_limit;
    uint16_t        shooter_speed_limit;
    uint16_t        shooter_heat;
		uint16_t        shooter_heat2;
	  uint16_t        trigger_last_angle;
    uint16_t        trigger_angle;
    uint16_t        shooter_heat_cooling_rate_k;		
		
} ShootHandle_t;

/* 宏定义 --------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void ShootTaskInit(void);

#endif

