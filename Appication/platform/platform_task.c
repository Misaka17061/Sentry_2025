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
#include "ramp.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
osThreadId PlatformTaskHandle;

/* 扩展变量 ------------------------------------------------------------------*/
extern PlatformHandle_t platform_handle;
extern fp32 ecd_ratio;

/* 私有函数原形 --------------------------------------------------------------*/
static void PlatformSensorUpdata(void);
static void PlatformCtrlModeSwitch(void);
static void PlatformInitMode(void);
static void PlatformFollowGimbalMode(void);
static void PlatformComputerRadarCtrlMode(void);
static void PlatformHumanCtrlMode(void);

/* 函数体 --------------------------------------------------------------------*/
void PlatformTask(void const*argument)
{  
    for(;;)
    {
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
            case COMPUTER_RADAR_CONTROL:
            {
                PlatformComputerRadarCtrlMode();
            }break;		
			case HUMAN_CONTROL:
            {
                PlatformHumanCtrlMode();
            }break;	
            default:
                break;
        }
				
		PlatformMotorControl(&platform_handle.yaw_motor);
				
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
		platform_handle.yaw_motor.sensor.relative_angle = ecd_ratio * (fp32)Motor_9025_RelativePosition(platform_handle.yaw_motor.motor_info->encoder,
                                                      			platform_handle.yaw_motor.encoderOffset);
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
		else if(platform_handle.console->platform_cmd == COMPUTER_RADAR_CONTROL_CMD)
		{
				platform_handle.ctrl_mode = COMPUTER_RADAR_CONTROL;
		}
		else if(platform_handle.console->platform_cmd == COMPUTER_VISION_CONTROL_CMD)
		{
				platform_handle.ctrl_mode = COMPUTER_VISION_CONTROL;
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
		
		platform_handle.yaw_motor.given_value = 0;//platform_handle.yaw_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&platform_ramp));

		if (fabsf(platform_handle.yaw_motor.sensor.relative_angle) <= 2.0f)
		{
			platform_handle.ctrl_mode = HUMAN_CONTROL;
		}
}

static void PlatformFollowGimbalMode(void)
{   
		platform_handle.yaw_motor.mode = GYRO_MODE;
		fp32 yaw_target = 0;
		yaw_target = platform_handle.yaw_motor.given_value ;
		platform_handle.yaw_motor.given_value = AngleTransform(yaw_target, platform_handle.yaw_motor.sensor.gyro_angle);
}

fp32 Platform_MOVE_RATIO = 0.2;
fp32 Platform_cruise_RATIO = 1;
fp32 begin_time = 0;
static void PlatformComputerRadarCtrlMode(void)
{
	ext_robot_hurt_t* robot_hurt_t = RefereeSystem_HeatData_Pointer();
	uint8_t armor_id = robot_hurt_t->armor_id;
	
	platform_handle.yaw_motor.mode = GYRO_MODE;
	Comm_GimbalInfo_t* info = GimbalInfo_Pointer();
	Comm_NavCmd_t* NavCmd_info = NavCmd_Pointer();
	fp32 yaw_target = 0;
//		if(platform_handle.console->under_attack == 0)
//		{
			if(info->vision_state == 0)
			{
					yaw_target = NavCmd_info->vw * Platform_cruise_RATIO + platform_handle.yaw_motor.given_value;
			}
			else
			{
					yaw_target = platform_handle.yaw_motor.given_value;
					if(platform_handle.gimbal_yaw_ecd_angle > 60)
					{
						 yaw_target = platform_handle.yaw_motor.given_value + 1;
					}
					else if(platform_handle.gimbal_yaw_ecd_angle < -60)
					{
						 yaw_target = platform_handle.yaw_motor.given_value - 1;
					}
			}
			
//		}
//		else
//			yaw_target = platform_handle.yaw_motor.sensor.gyro_angle - platform_handle.yaw_motor.sensor.relative_angle + armor_id * 90;
	platform_handle.yaw_motor.given_value = AngleTransform(yaw_target, platform_handle.yaw_motor.sensor.gyro_angle);
}

static void PlatformHumanCtrlMode(void)
{
	platform_handle.yaw_motor.mode = GYRO_MODE;
	fp32 yaw_target = 0;
	yaw_target = platform_handle.console->platform.yaw_v*3 + platform_handle.yaw_motor.given_value;
	platform_handle.yaw_motor.given_value = AngleTransform(yaw_target, platform_handle.yaw_motor.sensor.gyro_angle);
}
