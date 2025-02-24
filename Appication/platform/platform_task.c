/* 包含头文件 ----------------------------------------------------------------*/
#include "platform_task.h"
#include "cmsis_os.h"
#include "sentry_def.h"
#include "platform_app.h"
#include "timer_task.h"
#include "gimbal_app.h"
#include "gimbal_function.h"
#include "ESO.h"
#include "user_lib.h"
#include "user_protocol.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId PlatformTaskHandle;
extern PlatformHandle_t platform_handle;
extern pid_t platform_pid;
/* 扩展变量 ------------------------------------------------------------------*/
			 
/* 私有函数原形 --------------------------------------------------------------*/
static void PlatformSensorUpdata(void);
static void PlatformCtrlModeSwitch(void);
static void PlatformInitMode(void);
static void PlatformFollowGimbalMode(void);
static void PlatformComputerCtrlMode(void);
static void PlatformHumanCtrlMode(void);

/* 函数体 --------------------------------------------------------------------*/
void PlatformTask(void const*argument)
{  
    for(;;)
    {
				 ESO_calc(&ESO_9025,platform_handle.yaw_motor.powerControl);
				 PlatformSensorUpdata();
				 PlatformCtrlModeSwitch();
        switch (platform_handle.ctrl_mode)
        {
            case PLATFORM_INIT:
            {
                PlatformInitMode();
            }break;
						case PLATFORM_FOLLOW_GIMBAL:
            {
                PlatformFollowGimbalMode();
            }break;
            case COMPUTER_CONTROL:
            {
                PlatformComputerCtrlMode();
            }break;		
						case HUMAN_CONTROL:
            {
                PlatformHumanCtrlMode();
            }break;	
            default:
                break;
        }
				
				PlatformMotorControl(&platform_handle.yaw_motor);
				
				SoftwareTimerRegister(ESO_calc, (void*)NULL, 2);
				
				if (platform_handle.ctrl_mode == PLATFORM_RELAX)
        {
            dpid_clear(&platform_handle.yaw_motor.dpid);
            platform_handle.yaw_motor.powerControl = 0;
        }
				
				Torque_Control(platform_handle.platform_can ,0x141, platform_handle.yaw_motor.powerControl);
				
				

        osDelay(PLATFORM_TASK_PERIOD);
    }
}

static void PlatformSensorUpdata(void)
{
//    Encoder_Read(platform_handle.platform_can,PLATFORM_MOTOR_YAW_MESSAGE_ID);
//		osDelay(1);
//		Angle_Read(platform_handle.platform_can,PLATFORM_MOTOR_YAW_MESSAGE_ID);
//		osDelay(1);
//		platform_handle.yaw_motor.sensor.relative_angle = (((fp32)platform_handle.yaw_motor.motor_info->encoder - 11174.0f) / 32767.0f * 360.0f);
		ANGLE_LIMIT_180(platform_handle.yaw_motor.sensor.relative_angle,((fp32)platform_handle.yaw_motor.motor_info->encoder - 11174.0f) / 32767.0f * 360.0f);
		platform_handle.yaw_motor.sensor.gyro_angle = platform_handle.imu->attitude.yaw;
		platform_handle.yaw_motor.sensor.palstance = -platform_handle.imu->gyro[1] * RAD_TO_ANGLE;
		platform_handle.gimbal_yaw_ecd_angle = GimbalInfo_Pointer()->yaw_ecd_angle;
}

static void PlatformCtrlModeSwitch(void)
{
		platform_handle.last_ctrl_mode = platform_handle.ctrl_mode;

		if(platform_handle.console->platform_cmd == PLATFORM_INIT_CMD)
		{
				platform_handle.ctrl_mode = PLATFORM_INIT;
		}
		else if(platform_handle.console->platform_cmd == PLATFORM_FOLLOW_GIMBAL_CMD)
		{
				platform_handle.ctrl_mode = PLATFORM_FOLLOW_GIMBAL;
		}
		else if(platform_handle.console->platform_cmd == COMPUTER_CONTROL_CMD)
		{
				platform_handle.ctrl_mode = COMPUTER_CONTROL;
		}
		else if(platform_handle.console->platform_cmd == HUMAN_CONTROL_CMD)
		{
				platform_handle.ctrl_mode = HUMAN_CONTROL;
		}
		else if(platform_handle.console->platform_cmd == PLATFORM_RELEASE_CMD)
		{
				platform_handle.ctrl_mode = PLATFORM_RELAX;
		}

}

void PlatformTaskInit(void)
{
		ESO_9025_init();
    osThreadDef(platform_task, PlatformTask, osPriorityNormal, 0, 256);
    PlatformTaskHandle = osThreadCreate(osThread(platform_task), NULL);
}

static void PlatformInitMode(void)
{
		platform_handle.yaw_motor.mode = ENCONDE_MODE;
		fp32 yaw_target = 0;
		//Motor_On(platform_handle.platform_can,PLATFORM_MOTOR_YAW_MESSAGE_ID);
//		yaw_target = platform_handle.yaw_motor.sensor.relative_angle + platform_handle.yaw_motor.given_value;
//		platform_handle.yaw_motor.given_value = AngleTransform(yaw_target, platform_handle.yaw_motor.sensor.gyro_angle);
		platform_handle.yaw_motor.given_value = 0;
		if (fabsf(platform_handle.yaw_motor.sensor.relative_angle) <= 2.0f)
		platform_handle.ctrl_mode = HUMAN_CONTROL;
}

static void PlatformFollowGimbalMode(void)
{
		platform_handle.yaw_motor.given_value = pid_calc(&platform_handle.follow_pid,-platform_handle.gimbal_yaw_ecd_angle,0);
		platform_handle.yaw_motor.given_value -= ESO_9025.z1;
		platform_handle.yaw_motor.powerControl = ESO_DoublePID_Calc(&platform_handle.yaw_motor.dpid,
																														platform_handle.yaw_motor.given_value,
																														platform_handle.yaw_motor.sensor.gyro_angle,
																														platform_handle.yaw_motor.sensor.palstance,
																														&ESO_9025);
		Torque_Control(&can1_obj,0x141,platform_handle.yaw_motor.powerControl);
}

fp32 Platform_MOVE_RATIO = 0.1;
static void PlatformComputerCtrlMode(void)
{
		Comm_NavCmd_t* NavCmd_info = NavCmd_Pointer();
		fp32 yaw_target = 0;
		yaw_target = NavCmd_info->vw * Platform_MOVE_RATIO + platform_handle.yaw_motor.given_value;
		platform_handle.yaw_motor.given_value = AngleTransform(yaw_target, platform_handle.yaw_motor.sensor.gyro_angle);
		platform_handle.yaw_motor.given_value -= ESO_9025.z1;
		platform_handle.yaw_motor.powerControl = ESO_DoublePID_Calc(&platform_handle.yaw_motor.dpid,
																														platform_handle.yaw_motor.given_value,
																														platform_handle.yaw_motor.sensor.gyro_angle,
																														platform_handle.yaw_motor.sensor.palstance,
																														&ESO_9025);
		Torque_Control(&can1_obj,0x141,platform_handle.yaw_motor.powerControl);
}


static void PlatformHumanCtrlMode(void)
{
		fp32 yaw_target = 0;
		yaw_target = platform_handle.console->platform.yaw_v*3 + platform_handle.yaw_motor.given_value;
		platform_handle.yaw_motor.given_value = AngleTransform(yaw_target, platform_handle.yaw_motor.sensor.gyro_angle);
}
