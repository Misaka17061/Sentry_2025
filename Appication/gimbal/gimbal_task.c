/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "shoot_task.h"
#include "sentry_def.h"
#include "cmsis_os.h"
#include "computer_protocol.h"
#include "detect_task.h"
#include "referee_system.h"
#include "timer_task.h"
#include "user_protocol.h"
#include "ramp.h"
#include "ESO.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define BACK_CENTER_TIME 500
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

#define vision_comps_yaw 0                                    
#define vision_comps_pitch 1
/* 私有变量 ------------------------------------------------------------------*/
osThreadId GimbalTaskHandle;

static ramp_v0_t yaw_ramp = RAMP_GEN_DAFAULT;
static ramp_v0_t pitch_ramp = RAMP_GEN_DAFAULT;
VisionDatabase_t vision_data;
static fp32  speed_head = 0.1f;
static fp32	 yaw_speed = -0.1f;
/* 扩展变量 ------------------------------------------------------------------*/
extern GimbalHandle_t gimbal_handle;
extern pid_t yaw_vision_pid;
extern pid_t pitch_vision_pid;
extern AutoAim_t yaw_aim;
extern uint32_t diff_sof_time;
/* 私有函数原形 --------------------------------------------------------------*/
static void GimbalSensorUpdata(void);
static void GimbalCtrlModeSwitch(void);
static void GimbalMotorSendCurrent(int16_t yaw, int16_t pitch);
static void GimbalInitMode(void);
static void GimbalFollowPlatformMode(void);
static void GimbalNormalMode(void);
static void GimbalVisionAimMode(void);
static void GimbalSentryMode(void);
static void singal_level_debugging(uint16_t speed, fp32 angle_limit);
static void VisionDataUpdate(void);
static void pid_paramSA(AutoAim_t* Aim);
static void AimContorlModeSwitch(AutoAim_t* Aim);
static void Aim_contorl(AutoAim_t* Aim);

VisionDatabase_t* VisionData_Pointer(void);

uint16_t speed_= 10;
fp32 limit= 70;

/* 函数体 --------------------------------------------------------------------*/
void GimbalTask(void const*argument)
{
    for(;;)
    {
        GimbalSensorUpdata();
		VisionDataUpdate(); 	//处理视觉数据
				
        GimbalCtrlModeSwitch();
        switch (gimbal_handle.ctrl_mode)
        {
            case GIMBAL_INIT:
            {
                GimbalInitMode();
            }break;
			case GIMBAL_FOLLOW_PLATFORM:
            {
                GimbalFollowPlatformMode();							
            }break;
            case GIMBAL_NORMAL:
            {
				 GimbalNormalMode();
            }break;				
			case GIMBAL_VISION_AIM:
            {
                GimbalVisionAimMode();              
            }break;     
			case GIMBAL_SENTRY:
			{
				GimbalSentryMode();
			}break;
            default:
                break;
        }

        GimbalMotorControl(&gimbal_handle.yaw_motor);
        GimbalMotorControl(&gimbal_handle.pitch_motor);

        if (gimbal_handle.ctrl_mode == GIMBAL_RELAX)
        {
            dpid_clear(&gimbal_handle.yaw_motor.dpid);
            dpid_clear(&gimbal_handle.pitch_motor.dpid);
            gimbal_handle.yaw_motor.current_set = 0;
            gimbal_handle.pitch_motor.current_set = 0;
        }
        
       SoftwareTimerRegister(ESO_calc, (void*)NULL, 2);
				
       GimbalMotorSendCurrent((int16_t)YAW_MOTO_POSITIVE_DIR * gimbal_handle.yaw_motor.current_set,
                              (int16_t)PITCH_MOTO_POSITIVE_DIR * gimbal_handle.pitch_motor.current_set);
        osDelay(GIMBAL_TASK_PERIOD);

    }
}

void GimbalTaskInit(void)
{
	ESO_6020_init();
		
    osThreadDef(gimbal_task, GimbalTask, osPriorityNormal, 0, 256);
    GimbalTaskHandle = osThreadCreate(osThread(gimbal_task), NULL);
}

static void GimbalSensorUpdata(void)
{
    gimbal_handle.yaw_motor.sensor.relative_angle = gimbal_handle.yaw_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.yaw_motor.motor_info->ecd,
                                                      		 gimbal_handle.yaw_motor.offset_ecd);
    gimbal_handle.pitch_motor.sensor.relative_angle = gimbal_handle.pitch_motor.ecd_ratio * (fp32)Motor_RelativePosition(gimbal_handle.pitch_motor.motor_info->ecd,
                                                             gimbal_handle.pitch_motor.offset_ecd); 

    gimbal_handle.yaw_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.yaw;
    gimbal_handle.pitch_motor.sensor.gyro_angle = gimbal_handle.imu->attitude.pitch;
    gimbal_handle.yaw_motor.sensor.palstance = gimbal_handle.imu->gyro[2] * RAD_TO_ANGLE;
    gimbal_handle.pitch_motor.sensor.palstance = gimbal_handle.imu->gyro[0] * RAD_TO_ANGLE;
		
	gimbal_handle.platform_yaw_ecd_angle = PlatformInfo_Pointer()->yaw_ecd_angle;
	gimbal_handle.platform_yaw_gyro_angle = PlatformInfo_Pointer()->gyro_angle;
		
}

static void GimbalCtrlModeSwitch(void)
{
    gimbal_handle.last_ctrl_mode = gimbal_handle.ctrl_mode;
    if (gimbal_handle.console->gimbal_cmd == GIMBAL_RELEASE_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_RELAX;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_INIT_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_INIT;
    }
		else if (gimbal_handle.console->gimbal_cmd == GIMBAL_FOLLOW_PLATFORM_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_FOLLOW_PLATFORM;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_NORMAL_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
    }
    else if (gimbal_handle.console->gimbal_cmd == GIMBAL_VISION_AIM_CMD)
    {
        gimbal_handle.ctrl_mode = GIMBAL_VISION_AIM;
    }
	else if (gimbal_handle.console->gimbal_cmd == GIMBAL_SENTRY_CMD )	
	{
		gimbal_handle.ctrl_mode = GIMBAL_SENTRY;
	}	
}

static void GimbalMotorSendCurrent(int16_t yaw_cur, int16_t pitch_cur)
{
    Motor_SendMessage(gimbal_handle.gimbal_can, GIMBAL_MOTOR_CONTROL_STD_ID, yaw_cur, pitch_cur, 0, 0);
}

static void GimbalInitMode(void)
{
    if(gimbal_handle.last_ctrl_mode != GIMBAL_INIT)
    {
        ramp_v0_init(&yaw_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
        ramp_v0_init(&pitch_ramp, BACK_CENTER_TIME/GIMBAL_TASK_PERIOD);
    }

    gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;

    gimbal_handle.pitch_motor.given_value = gimbal_handle.pitch_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&pitch_ramp));
    if (fabsf(gimbal_handle.pitch_motor.sensor.relative_angle) <= 2.0f)
    {
        gimbal_handle.yaw_motor.given_value = gimbal_handle.yaw_motor.sensor.relative_angle * (1 - ramp_v0_calculate(&yaw_ramp));
		if (fabsf(gimbal_handle.yaw_motor.sensor.relative_angle) <= 3.0f )
        {
            gimbal_handle.ctrl_mode = GIMBAL_NORMAL;
        }
	}
}

static void GimbalFollowPlatformMode(void)
{
	gimbal_handle.yaw_motor.mode = ENCONDE_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
		
	gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
	gimbal_handle.yaw_motor.given_value = 0;

	VAL_LIMIT(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.min_relative_angle, gimbal_handle.yaw_motor.max_relative_angle);
    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
}

static void GimbalNormalMode(void)
{
    fp32 yaw_target=0;
    
    gimbal_handle.yaw_motor.mode = GYRO_MODE;
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
	
    yaw_target = gimbal_handle.yaw_motor.given_value + gimbal_handle.console->gimbal.yaw_v;

    gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.gyro_angle);
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v;
      
    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);
}

static void GimbalSentryMode(void)
{  
	fp32 yaw_target=0;
	Comm_NavCmd_t* NavCmd_info = NavCmd_Pointer();
	VisionDatabase_t* info = VisionData_Pointer();//更新视觉数据
	if (info->state)
	{
		gimbal_handle.yaw_motor.mode = GYRO_MODE;                                       //使能云台yaw轴陀螺仪模式
		gimbal_handle.pitch_motor.mode = ENCONDE_MODE;                                  //使能云台pitch轴编码器模式
		Aim_contorl(&yaw_aim);
		pid_paramSA(&yaw_aim);//自适应PID 
	
		info->yaw -= ESO_6020.z1;
		gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v+vision_comps_pitch*pid_calc(&pitch_vision_pid,gimbal_handle.pitch_motor.given_value,gimbal_handle.pitch_motor.given_value+info->pitch);
		gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v+pid_calc(&yaw_vision_pid,gimbal_handle.yaw_motor.given_value,gimbal_handle.yaw_motor.given_value+info->yaw);

		VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);//pitch角度限位
		gimbal_handle.yaw_motor.given_value  =  AngleTransform(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.sensor.gyro_angle);
	}
	else
	{
		gimbal_handle.yaw_motor.mode = ENCONDE_MODE;                                       
		gimbal_handle.pitch_motor.mode = ENCONDE_MODE;
				
		yaw_target = gimbal_handle.yaw_motor.given_value + yaw_speed;
		gimbal_handle.yaw_motor.given_value = AngleTransform(yaw_target, gimbal_handle.yaw_motor.sensor.relative_angle); 
		gimbal_handle.pitch_motor.given_value += speed_head;	
		VAL_ROLLBACK(yaw_target,-70,+70,yaw_speed,0.5f);	
		VAL_ROLLBACK(gimbal_handle.pitch_motor.given_value,-10,29,speed_head,0.4f);	
	}
}


static void GimbalVisionAimMode(void)
{   
    gimbal_handle.yaw_motor.mode = GYRO_MODE;                                       //使能云台yaw轴陀螺仪模式
    gimbal_handle.pitch_motor.mode = ENCONDE_MODE;                                  //使能云台pitch轴编码器模式

	Aim_contorl(&yaw_aim);
	pid_paramSA(&yaw_aim);//自适应PID 
	
	VisionDatabase_t* info = VisionData_Pointer();//更新视觉数据
    info->yaw -= ESO_6020.z1;
    gimbal_handle.pitch_motor.given_value += gimbal_handle.console->gimbal.pitch_v+vision_comps_pitch*pid_calc(&pitch_vision_pid,gimbal_handle.pitch_motor.given_value,gimbal_handle.pitch_motor.given_value+info->pitch);
    gimbal_handle.yaw_motor.given_value += gimbal_handle.console->gimbal.yaw_v+pid_calc(&yaw_vision_pid,gimbal_handle.yaw_motor.given_value,gimbal_handle.yaw_motor.given_value+info->yaw);

    VAL_LIMIT(gimbal_handle.pitch_motor.given_value, gimbal_handle.pitch_motor.min_relative_angle, gimbal_handle.pitch_motor.max_relative_angle);//pitch角度限位
    gimbal_handle.yaw_motor.given_value  =  AngleTransform(gimbal_handle.yaw_motor.given_value, gimbal_handle.yaw_motor.sensor.gyro_angle);
}

static void VisionDataUpdate(void)
{
    Comm_VisionInfo_t* info = VisionInfo_Pointer();
    if(CheckDeviceIsOffline(OFFLINE_VISION_INFO)){vision_data.pitch = vision_data.yaw  = 0;return;}
    vision_data.state = TwoBytesToInt(info->state);
	vision_data.can_shoot = TwoBytesToInt(info->can_shoot);
	vision_data.move_state = TwoBytesToInt(info->move_state);
	vision_data.last_move_state = vision_data.move_state;
    vision_data.last_pitch = vision_data.pitch;
	ESO_6020.y = vision_data.yaw - vision_data.last_yaw;
    vision_data.last_yaw = vision_data.yaw;
	vision_data.last_palstance = vision_data.palstance;
	vision_data.accelerated_speed = (vision_data.palstance - vision_data.last_palstance) / diff_sof_time;
		
    vision_data.pitch = FourBytesToFloat(info->pitch);   
    
    vision_data.yaw = FourBytesToFloat(info->yaw);
		
	vision_data.palstance = FourBytesToFloat(info->palstance) * 180 / PI;	
		
	yaw_aim.systeam_time = BSP_GetTime_ms();//获取代码总运行时间
	ESO_calc(&ESO_6020,gimbal_handle.yaw_motor.current_set);
}

static void AimContorlModeSwitch(AutoAim_t* Aim)
{
	if (vision_data.state)//视觉NUC在线时，此条件一直成立
	{     
		if(ABS(vision_data.yaw)<Aim->tol_angle)//自瞄已经达到一定角度之内
		{  
				//thought to be aimed right认为自瞄正确
			if (vision_data.yaw_success == AIM_NO&&!Aim->first_aim)
			{ 
				Aim->aim_mode = FIRST_AIMING;
			}
			else if(vision_data.yaw_success == AIM_NO&&Aim->first_aim)
			{
				Aim->aim_mode = COMPLETE_AIMING;
			}      
		}
		else
		{  
			Aim->aim_mode = FOLLOW;
		}
	}
	else
	{
		Aim->aim_mode = RELAX;
	}
}

static void Aim_contorl(AutoAim_t* Aim)
{
	AimContorlModeSwitch(Aim);
	switch (Aim->aim_mode)
    {
		case RELAX:
		{
		//视觉NUC离线，无法被视觉控制自瞄
		vision_data.yaw_success = AIM_NO;
		Aim->aiming_time = 0 , Aim->stay_time = 0;
        yaw_vision_pid.i=0.00f;		
		}break;
		case FOLLOW:
		{
		 //自瞄开始，此时还未瞄准到规定角度差
        vision_data.yaw_success = AIM_NO,yaw_vision_pid.p = 0.018;//user set
			   
		if (Aim->aim_flag == 0) 
		{
            Aim->Astart_time = Aim->systeam_time,Aim->aim_flag = 1;//记录自瞄开始时间，并进入新的单次自瞄         
		} 
		else
		{
            Aim->aiming_time = Aim->systeam_time - Aim->Astart_time;//记录自瞄持续时间
		}
		}break;
		case FIRST_AIMING:
		{
				/************记录未进入第一次自瞄的时间**********/  
        Aim->Sstart_time = Aim->systeam_time; 
        Aim->first_aim = 1, Aim->aiming_time = 0;//进入第一次自瞄
        yaw_vision_pid.p= 0.004;//user set   
		}break;
		case COMPLETE_AIMING:
		{
		Aim->stay_time = Aim->systeam_time-Aim->Sstart_time;//comparing aimed right time
        if(Aim->stay_time>Aim->tol_time)//自瞄持续时间达到tol_time的标准
		{
			Aim->first_aim = 0,Aim->aim_flag = 0;//认为已完成瞄准，退出此次自瞄及消抖 	
			if(ABS(vision_data.yaw)<0.1f)vision_data.yaw_success = AIM_RIGHT;//if long enough ,thought to be aimed stable 
		}
		}break;
		default:
        break;
	}
		VAL_LIMIT(vision_data.yaw,-15.0f,15.0f); 
}

VisionDatabase_t* VisionData_Pointer(void)
{
    return &vision_data;
}

static void pid_paramSA(AutoAim_t* Aim)
{
 if(Aim->enable_paramSA)
 {
  if(vision_data.yaw_success == AIM_NO)
	{
		/*时间豁度补偿系数P   TODO 增添I补偿及D补偿*/
		yaw_vision_pid.p += ((float)(Aim->aiming_time*0.0008/Aim->Ap_parm));
		if(Aim->first_aim)//如果进入第一次自瞄
		{
			VAL_LIMIT(Aim->stay_time,1,1000);
			yaw_vision_pid.p -= (Aim->Sp_parm/(float)(Aim->stay_time));
		}
		if(vision_data.yaw==-90)//90位规定的角度
		yaw_vision_pid.i=0;
		VAL_LIMIT(yaw_vision_pid.p,0.01f,0.1f);
		VAL_LIMIT(yaw_vision_pid.i, 0.0f, 0.000015f);
	}
 }
}

int label = 0;
static void singal_level_debugging(uint16_t speed, fp32 angle_limit)
{

		int speed_ = 0;
//			if(label == 0)
//			{
//					speed_ = speed;
//					if(gimbal_handle.yaw_motor.sensor.relative_angle > angle_limit)
//							label = 1;
//			}
//			else if(label == 1)
//			{
//					speed_ = -speed;
//					if(gimbal_handle.yaw_motor.sensor.relative_angle < -angle_limit)
//							label = 0;
//			}
//		
//		
//		gimbal_handle.yaw_motor.current_set = pid_calc(&gimbal_handle.yaw_motor.dpid.inter_pid, gimbal_handle.yaw_motor.sensor.palstance, speed_);
	
				if(label == 0)
			{
					speed_ = speed;
					if(gimbal_handle.pitch_motor.sensor.relative_angle > angle_limit)
							label = 1;
			}
			else if(label == 1)
			{
					speed_ = -speed;
					if(gimbal_handle.pitch_motor.sensor.relative_angle < -angle_limit)
							label = 0;
			}
		
		
		gimbal_handle.pitch_motor.current_set = pid_calc(&gimbal_handle.pitch_motor.dpid.inter_pid, gimbal_handle.pitch_motor.sensor.palstance, speed_);
}


