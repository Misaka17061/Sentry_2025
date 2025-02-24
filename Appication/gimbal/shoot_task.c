/* 包含头文件 ----------------------------------------------------------------*/
#include "shoot_task.h"
#include "sentry_def.h"
#include "cmsis_os.h"
#include "math.h"
#include "bsp_init.h"
#include "detect_task.h"
#include "referee_system.h"
#include "gimbal_app.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define TRIGGER_MOTOR_POSITIVE_DIR      (-1.0f)
#define TRIGGER_MOTOR_REDUCTION_RATIO   M2006_REDUCTION_RATIO
#define TRIGGER_PLATE_NUMBERS   (6.0f)
#define ONE_BULLET_HEAT         (10u)

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ShootTaskHandle;
ShootHandle_t shoot_handle;

static uint16_t shoot_speed_limit[2] = {15, 30};      
static uint16_t friction_wheel_speed[2] = {4500 , 8500};

ADRC  adrc[2];

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static void ShootSensorUpdata(void);
static void ShootCtrlModeSwitch(void);
static void Shoot_TriggerMotorCtrl(ShootHandle_t* handle);
static void Shoot_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, M3508_Motor_t motor[2]);
static void ShootMotorSendCurrent(int16_t fric1_cur, int16_t fric2_cur, int16_t trigger_cur, int16_t magazine_cur);
static void TriggerBackInit(Trigger_back_t* TriggerBack,uint32_t threshold, uint32_t backoff_time, fp32 backoff_angle);

/* 函数体 --------------------------------------------------------------------*/
void ShootTask(void const*argument)
{
    for(;;)
    {
        ShootSensorUpdata();
        ShootCtrlModeSwitch();
			
        Shoot_TriggerMotorCtrl(&shoot_handle);
        Shoot_FrictionWheelMotorCtrl(shoot_handle.ctrl_mode, shoot_handle.fric_wheel_motor);

        if (shoot_handle.ctrl_mode == SHOOT_RELAX)
        {
            shoot_handle.fric_wheel_motor[0].current_set = 0;
            shoot_handle.fric_wheel_motor[1].current_set = 0;
            shoot_handle.trigger_motor.current_set = 0;
        }
        ShootMotorSendCurrent(shoot_handle.fric_wheel_motor[0].current_set,
                              shoot_handle.fric_wheel_motor[1].current_set,
                              TRIGGER_MOTOR_POSITIVE_DIR * shoot_handle.trigger_motor.current_set,
                              0);
        
        osDelay(SHOOT_TASK_PERIOD);
    }
}

void ShootTaskInit(void)
{
    shoot_handle.console = Console_Pointer();
    shoot_handle.shoot_can = &can2_obj;
	

    shoot_handle.trigger_motor.motor_info = TriggerMotor_Pointer();
    shoot_handle.trigger_state = TRIGGER_END;
    shoot_handle.trigger_motor.ecd_ratio = TRIGGER_MOTOR_POSITIVE_DIR * TRIGGER_MOTOR_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
	
    pid_init(&shoot_handle.trigger_motor.dpid.outer_pid, POSITION_PID, 300.0f, 60.0f,
             14.0f, 0.0f, 20.5f);/*8，0，0*/  /*调速*//*慢加P，抖加D*//*拨弹盘*/  //8,0,25
    pid_init(&shoot_handle.trigger_motor.dpid.inter_pid, POSITION_PID, M2006_MOTOR_MAX_CURRENT, 7000.0f,
             100.0f, 0.0f, 0.0f);/*100，0，0*/ /*调力*/
	
		TriggerBackInit(&shoot_handle.trigger_back, 500, 500, 60);

    shoot_handle.fric_wheel_motor[0].motor_info = FrictionWheelMotor_1_Pointer();
    shoot_handle.fric_wheel_motor[1].motor_info = FrictionWheelMotor_2_Pointer();
    for (uint8_t i = 0; i < 2; i++)/*摩擦轮*/
    {
        pid_init(&shoot_handle.fric_wheel_motor[i].pid, POSITION_PID, 10000, 500.0f,
                 40.0f, 0.1f, 15.0f);/*9,0,0*/
//			        adrc[i]=zepi_create_ADRC();
//         _adrc_reset_allParam(adrc+i,Default_3508_speed);
    }

    osThreadDef(shoot_task, ShootTask, osPriorityNormal, 0, 256);
    ShootTaskHandle = osThreadCreate(osThread(shoot_task), NULL);
}

static void ShootSensorUpdata(void)
{
    shoot_handle.trigger_motor.speed = (fp32)shoot_handle.trigger_motor.motor_info->speed_rpm * TRIGGER_MOTOR_REDUCTION_RATIO * TRIGGER_MOTOR_POSITIVE_DIR;
    shoot_handle.trigger_motor.angle = shoot_handle.trigger_motor.ecd_ratio
            * (fp32)(shoot_handle.trigger_motor.motor_info->total_ecd - shoot_handle.trigger_motor.offset_ecd);

      
	
    if (!CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
    {
        shoot_handle.shooter_heat_cooling_rate = RefereeSystem_RobotState_Pointer()->shooter_id1_17mm_cooling_rate;
        shoot_handle.shooter_heat_cooling_limit = RefereeSystem_RobotState_Pointer()->shooter_id1_17mm_cooling_limit;			
			  shoot_handle.shooter_speed_limit = RefereeSystem_RobotState_Pointer()->shooter_id1_17mm_speed_limit;
        shoot_handle.shooter_heat = RefereeSystem_PowerHeatData_Pointer()->shooter_id1_17mm_cooling_heat;
			  shoot_handle.shooter_speedfeedback=RefereeSystem_SpeedData_Pointer()->bullet_speed;
		  	shoot_handle.shooter_heat_cooling_rate_k=shoot_handle.shooter_heat_cooling_rate;
    }
    else
    {
        shoot_handle.shooter_heat_cooling_rate = 0;
        shoot_handle.shooter_heat_cooling_limit = 200;
        shoot_handle.shooter_heat = 0;
				shoot_handle.shooter_speed_limit=30;
    }
}
static void ShootCtrlModeSwitch(void)
{
    if (shoot_handle.console->shoot_cmd == SHOOT_RELEASE_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_RELAX;
    }
    else if (shoot_handle.console->shoot_cmd == SHOOT_START_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_START;
    }
    else if (shoot_handle.console->shoot_cmd == SHOOT_STOP_CMD)
    {
        shoot_handle.ctrl_mode = SHOOT_STOP;
    }
}

int speed = 20;

static void Shoot_TriggerMotorCtrl(ShootHandle_t* handle)
{
    if (handle->ctrl_mode == SHOOT_START)           //开始射击模式
    {
				if (handle->trigger_state == TRIGGER_END)       //拨弹结束状态
        {
						if(handle->console->shoot.fire_cmd == ONE_FIRE_CMD)   //单发热量计算
						{
								handle->trigger_state = TRIGGER_BEGIN;
						}
						else if(handle->console->shoot.fire_cmd == RAPID_FIRE_CMD)
						{
								handle->trigger_state = RAPID_TRIGGERING;
						}
					
        }
        else if (handle->trigger_state == TRIGGER_BEGIN)        //开始拨弹模式
        {                                 
            handle->trigger_state = TRIGGERING;             //拨弹中状态
            handle->trigger_motor.set_angle = handle->trigger_motor.angle - (360.0f / TRIGGER_PLATE_NUMBERS);   //拨弹电机角度设定为拨弹电机当前角度加 360/TRIGGER_PLATE_NUMBERS
						handle->trigger_back.start_time = BSP_GetTime_ms();

        }
        else if (handle->trigger_state == TRIGGERING)           //拨弹中模式
        {
            if ( fabs(handle->trigger_motor.set_angle - handle->trigger_motor.angle) < 3.0f )        
            {                                                       
                handle->trigger_state = TRIGGER_END;                                            
            }
						else if(BSP_GetTime_ms() > (handle->trigger_back.start_time + handle->trigger_back.threshold))
						{
								handle->trigger_state = TRIGGER_BACKING;            
								handle->trigger_motor.set_angle = handle->trigger_motor.angle + handle->trigger_back.backoff_angle; 
								handle->trigger_back.start_time = BSP_GetTime_ms();
						}
        }
				else if(handle->trigger_state == RAPID_TRIGGERING)
				{
				   handle->trigger_motor.set_angle = handle->trigger_motor.angle - speed;
					 handle->trigger_back.start_time = BSP_GetTime_ms();
					 if (BSP_GetTime_ms() > (handle->trigger_back.start_time + handle->trigger_back.threshold))
					 {
							if(handle->trigger_motor.motor_info->speed_rpm < 5000)
							{
									handle->trigger_state = TRIGGER_BACKING;            
									handle->trigger_motor.set_angle = handle->trigger_motor.angle + handle->trigger_back.backoff_angle;   
									handle->trigger_back.start_time = BSP_GetTime_ms();
							}
					 }
					 else if(handle->console->shoot.fire_cmd == STOP_FIRE_CMD)
					 {
							handle->trigger_state = TRIGGER_END; 
					 }
				}
				else if(handle->trigger_state == TRIGGER_BACKING)
				{
            if ( fabs(handle->trigger_motor.set_angle - handle->trigger_motor.angle) < 3.0f )        
            {                                                        
                handle->trigger_state = TRIGGER_BEGIN;                                               //拨弹开始
            }
						else if(BSP_GetTime_ms() > (handle->trigger_back.start_time + handle->trigger_back.backoff_time))
						{                                                      
                handle->trigger_state = TRIGGER_END;

						}
				
				}
    }
    else
    {
        handle->trigger_state = TRIGGER_END;                                                        //拨弹结束
        handle->trigger_motor.set_angle = handle->trigger_motor.angle;                             
    }
		

		
    handle->trigger_motor.current_set = DoublePID_Calc(&handle->trigger_motor.dpid,                  
                                                       handle->trigger_motor.set_angle,
                                                       handle->trigger_motor.angle,
                                                       handle->trigger_motor.speed);
}

static void Shoot_FrictionWheelMotorCtrl(ShootCtrlMode_e mode, M3508_Motor_t motor[2])
{
    if (mode == SHOOT_START)
    {
        for (uint8_t i = 0; i<2; i++)
        {
            if (shoot_speed_limit[i]  == 15)
            {
                motor[0].given_speed = friction_wheel_speed[i] ;
                motor[1].given_speed = -friction_wheel_speed[i] ;
            }
        }
    }
    else
    {
        motor[0].given_speed = 0;
        motor[1].given_speed = 0;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        motor[i].current_set = pid_calc(&motor[i].pid, motor[i].motor_info->speed_rpm, motor[i].given_speed);
//          motor[i].current_set=_adrc_getOutput(adrc+i,
//		 									motor[i].given_speed,
//		 									motor[i].motor_info->speed_rpm);  			
    }
}

static void TriggerBackInit(Trigger_back_t* TriggerBack,uint32_t threshold, uint32_t backoff_time, fp32 backoff_angle)
{
		TriggerBack->threshold = threshold;
		TriggerBack->backoff_time = backoff_time;
		TriggerBack->backoff_angle = backoff_angle;
}

static void ShootMotorSendCurrent(int16_t fric1_cur, int16_t fric2_cur, int16_t trigger_cur, int16_t magazine_cur)
{
    Motor_SendMessage(shoot_handle.shoot_can, SHOOT_MOTOR_CONTROL_STD_ID, fric1_cur, fric2_cur, trigger_cur, magazine_cur);
}
