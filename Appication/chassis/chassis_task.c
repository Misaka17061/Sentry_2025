/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_task.h"
#include "sentry_def.h"
#include "cmsis_os.h"
#include "chassis_function.h"
#include "referee_system.h"
#include "computer_protocol.h"
#include "user_protocol.h"
#include <stdio.h>
#include <math.h>

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ChassisTaskHandle;

/* 扩展变量 ------------------------------------------------------------------*/
extern ChassisHandle_t chassis_handle;

/* 私有函数原形 --------------------------------------------------------------*/
static void ChassisSensorUpdata(void);
static void ChassisCtrlModeSwitch(void);

static void ChassisFollowGimbalMode(void);
static void ChassisSeparateGimbalMode(void);
static void ChassisSpinMode(void);
static void ChassisSentryMode(void);

static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur);

/* 函数体 --------------------------------------------------------------------*/
void Chassis_Task(void const *argument)
{
	for(;;)
	{
		ChassisSensorUpdata();
		ChassisCtrlModeSwitch();
		switch (chassis_handle.ctrl_mode)
        {
			case CHASSIS_FOLLOW_GIMBAL:
			ChassisFollowGimbalMode();
			break;							
			case CHASSIS_SEPARATE_GIMBAL:
			ChassisSeparateGimbalMode();
			break;							
			case  CHASSIS_SPIN:
			ChassisSpinMode();
			break;
			case CHASSIS_SENTRY:
			ChassisSentryMode();
			break;
			default:
			break;
		}
				
		Chassis_ControlCalc(&chassis_handle);          //底盘解算

        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_handle.chassis_motor[i].given_speed = chassis_handle.wheel_rpm[i];
            chassis_handle.chassis_motor[i].current_set = pid_calc(&chassis_handle.chassis_motor[i].pid,
                                                                   chassis_handle.chassis_motor[i].motor_info->speed_rpm,
                                                                   chassis_handle.chassis_motor[i].given_speed);
        }

		    Chassis_LimitPower(&chassis_handle);    //底盘功率限制
				
       if(chassis_handle.ctrl_mode == CHASSIS_RELAX)
        {
            for (uint8_t i = 0; i < 4; i++)
            {
                chassis_handle.chassis_motor[i].current_set = 0;
            }
        }
				
	      ChassisMotorSendCurrent(chassis_handle.chassis_motor[0].current_set,
                                    chassis_handle.chassis_motor[1].current_set,
                                    chassis_handle.chassis_motor[2].current_set,
                                    chassis_handle.chassis_motor[3].current_set);

				osDelay(CHASSIS_TASK_PERIOD);
		}
}

void ChassisTaskInit(void)   //任务初始化 
{	
    osThreadDef(chassis_task, Chassis_Task, osPriorityNormal, 0, 256);
    ChassisTaskHandle = osThreadCreate(osThread(chassis_task), NULL);
}

static void ChassisSensorUpdata(void)
{
    chassis_handle.platform_yaw_ecd_angle = PlatformInfo_Pointer()->yaw_ecd_angle;
	chassis_handle.gimbal_yaw_ecd_angle = GimbalInfo_Pointer()->yaw_ecd_angle;
}

static void ChassisCtrlModeSwitch(void)
{
    if (chassis_handle.console->chassis_cmd == CHASSIS_RELEASE_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_RELAX;
    }		
    else if (chassis_handle.console->chassis_cmd == CHASSIS_STOP_CMD)
    {
        chassis_handle.ctrl_mode = CHASSIS_STOP;
    }
	else if (chassis_handle.console->chassis_cmd == CHASSIS_SEPARATE_GIMBAL_CMD)
	{
		chassis_handle.ctrl_mode = CHASSIS_SEPARATE_GIMBAL;
	}
	else if (chassis_handle.console->chassis_cmd == CHASSIS_FOLLOW_GIMBAL_CMD)
	{
	   	chassis_handle.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
	}
	else if (chassis_handle.console->chassis_cmd == CHASSIS_SPIN_CMD)
	{
		chassis_handle.ctrl_mode = CHASSIS_SPIN;
	}
	else if (chassis_handle.console->chassis_cmd == CHASSIS_NAV_CMD)
	{
		chassis_handle.ctrl_mode = CHASSIS_SENTRY;
	}
}

static void ChassisFollowGimbalMode(void)
{   
    chassis_handle.vx = chassis_handle.console->chassis.vx;
    chassis_handle.vy = chassis_handle.console->chassis.vy;
    chassis_handle.vw = pid_calc(&chassis_handle.follow_pid,
                                 -chassis_handle.platform_yaw_ecd_angle,
	                               0); 
}

static void ChassisSeparateGimbalMode(void)
{
	chassis_handle.vx = chassis_handle.console->chassis.vx;
	chassis_handle.vy = chassis_handle.console->chassis.vy;
	chassis_handle.vw = 0;
}

static void ChassisSpinMode(void)
{
	chassis_handle.vx = chassis_handle.console->chassis.vx * 2;
	chassis_handle.vy = chassis_handle.console->chassis.vy * 2;
	
	chassis_handle.vw = 90;
}

static void ChassisSentryMode(void)
{
	Comm_NavCmd_t* NavCmd_info = NavCmd_Pointer();
	chassis_handle.vx = NavCmd_info->vx * 600;
	chassis_handle.vy = -NavCmd_info->vy * 600;
    if(chassis_handle.vx < 0.15 && chassis_handle.vy == 0)   
		chassis_handle.vw = 103.5;		//rpm75
	else
		chassis_handle.vw = 90;
}

static void ChassisMotorSendCurrent(int16_t motor1_cur, int16_t motor2_cur, int16_t motor3_cur, int16_t motor4_cur)  
{
    Motor_SendMessage(chassis_handle.chassis_can, MOTOR_1TO4_CONTROL_STD_ID, motor1_cur, motor2_cur, motor3_cur, motor4_cur);
}

