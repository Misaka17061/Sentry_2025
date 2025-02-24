/* 包含头文件 ----------------------------------------------------------------*/
#include "sentry_console.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "sentry_def.h"
#include "user_protocol.h"
#include "referee_system.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define SPIN_RATE 300

/* 私有变量 ------------------------------------------------------------------*/
osThreadId ConsoleTaskHandle;

//static WBUS_info_t* rc;

/* 扩展变量 ------------------------------------------------------------------*/
Console_t console;
RC_Switch_t wheel_switch;
fp32 shoot_start_time;

//WBUS_Switch_t WBUS_switch;
extern GimbalHandle_t gimbal_handle;
extern VisionDatabase_t vision_data;
extern ext_robot_hurt_t robot_hurt_t;
/* 私有函数原形 --------------------------------------------------------------*/
static void Gimbal_RemoteControl_Operation(void);
static void Platform_RemoteControl_Operation(void);
static void ComputerControl_Operation(void);   
static void RemoteControlWheelAction(void);
/* 函数体 --------------------------------------------------------------------*/
void ConsoleTask(void const* argument)
{
	

	for(;;)
	{    
		RemoteControlWheelAction();
		switch(console.ctrl_mode)
		{
			case PREPARE_MODE:
			{
                if(GimbalInfo_Pointer()->mode != GIMBAL_RELAX &&
                    GimbalInfo_Pointer()->mode != GIMBAL_INIT &&
										PlatformInfo_Pointer()->mode != PLATFORM_INIT)
                {
                    console.ctrl_mode = NORMAL_MODE;
										console.gimbal_cmd = GIMBAL_NORMAL_CMD;
                    console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
                    console.shoot_cmd = SHOOT_STOP_CMD;
										console.platform_cmd = HUMAN_CONTROL_CMD;
                }
                else
                {
                    console.ctrl_mode = PREPARE_MODE;
                    console.gimbal_cmd = GIMBAL_INIT_CMD;
                    console.chassis_cmd = CHASSIS_STOP_CMD;
										console.platform_cmd = HUMAN_CONTROL_CMD;
										console.platform_cmd = PLATFORM_INIT_CMD;
                }				
			}break;

			case NORMAL_MODE:
			{
//                if (rc->toggle_switch.SA == REMOTE_ET_SWITCH_VALUE_CENTRAL)
//                {
//                    RemoteControl_Operation();    
//                }
//                else if(rc->toggle_switch.SA == REMOTE_ET_SWITCH_VALUE_UP)
//                {
//                    ComputerControl_Operation();
//                }
//                else if(rc->toggle_switch.SA == REMOTE_ET_SWITCH_VALUE_DOWN)
//                {
//                
//                }
								if (console.rc->sw1 == REMOTE_SWITCH_VALUE_CENTRAL)
                {
										Platform_RemoteControl_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_UP)
                {
                    ComputerControl_Operation();
                }
                else if(console.rc->sw1 == REMOTE_SWITCH_VALUE_DOWN)
                {
										Gimbal_RemoteControl_Operation(); 
                }
			}break;

			case SAFETY_MODE:
			{
//                if(!CheckDeviceIsOffline(OFFLINE_WBUS))
								if(!CheckDeviceIsOffline(OFFLINE_DBUS))
                {
                    console.ctrl_mode = PREPARE_MODE;
                }
                else
                {
                    console.gimbal_cmd  = GIMBAL_RELEASE_CMD;
                    console.chassis_cmd  = CHASSIS_RELEASE_CMD;
										console.platform_cmd = PLATFORM_RELEASE_CMD;
                    console.shoot_cmd = SHOOT_RELEASE_CMD;
                }
			}break;

			default:
				break;
		}

//        if(CheckDeviceIsOffline(OFFLINE_WBUS))
				if(CheckDeviceIsOffline(OFFLINE_DBUS))
        {
            console.ctrl_mode = SAFETY_MODE;
        }

        osDelay(CONSOLE_TASK_PERIOD);
    }	

}	

void ConsoleTaskInit(void)
{
//	rc = WBUS_GetDataPointer();
	console.rc = RC_GetDataPointer();

	console.ctrl_mode = PREPARE_MODE;
	console.gimbal_cmd = GIMBAL_INIT_CMD;
	console.chassis_cmd = CHASSIS_STOP_CMD;
	console.shoot_cmd = SHOOT_STOP_CMD;
	console.shoot.fire_cmd = STOP_FIRE_CMD;
	
	osThreadDef(console_task, ConsoleTask, osPriorityNormal, 0, 256);
  ConsoleTaskHandle = osThreadCreate(osThread(console_task), NULL);
}

static void RemoteControlWheelAction(void)
{
    static uint8_t wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    if (console.rc->wheel < -440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_UP;
    }
    else if (console.rc->wheel > -220 && console.rc->wheel < 220)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_CENTRAL;
    }
    else if (console.rc->wheel > 440)
    {
        wheel_sw = REMOTE_SWITCH_VALUE_DOWN;
    }
    RC_SwitchAction(&wheel_switch, wheel_sw);
		
}

//static void RemoteControl_Operation(void)
//{ 
//		RC_ET_SwitchAction(&WBUS_switch, rc->toggle_switch.SH);
//    if (rc->toggle_switch.SD == REMOTE_ET_SWITCH_VALUE_CENTRAL)           //云台底盘分离      
//    {
//				console.gimbal_cmd = GIMBAL_VISION_AIM_CMD;
//        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
//				console.platform_cmd = HUMAN_CONTROL_CMD;

//        console.chassis.vx = (rc->Ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
//        console.chassis.vy = -(rc->Ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y);
//				console.gimbal.pitch_v = rc->Ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
//        if(rc->toggle_switch.SE == REMOTE_ET_SWITCH_VALUE_DOWN)
//				{
//					console.platform.yaw_v = 0;
//					console.gimbal.yaw_v = -rc->Ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
//				}
//				else if(rc->toggle_switch.SE == REMOTE_ET_SWITCH_VALUE_UP)
//				{
//					console.platform.yaw_v = -rc->Ch1 * ratio;
//					console.gimbal.yaw_v = 0;
//				}
//    }
//		
//    else if(rc->toggle_switch.SD == REMOTE_ET_SWITCH_VALUE_DOWN)          //小陀螺
//    {
//        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
//        console.chassis_cmd = CHASSIS_SPIN_CMD;
//				console.platform_cmd = HUMAN_CONTROL_CMD;
//			
//        console.chassis.vx = -rc->Ch4 / -RC_RESOLUTION * 300 + 6;
//        console.chassis.vy = rc->Ch3 / -RC_RESOLUTION * 300 + 5;
//        console.gimbal.pitch_v = rc->Ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
//        if(rc->toggle_switch.SE == REMOTE_ET_SWITCH_VALUE_DOWN)
//				{
//					console.platform.yaw_v = 0;
//					console.gimbal.yaw_v = -rc->Ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
//				}
//				else if(rc->toggle_switch.SE == REMOTE_ET_SWITCH_VALUE_UP)
//				{
//					console.platform.yaw_v = -rc->Ch1 * ratio;
//					console.gimbal.yaw_v = 0;
//				}
//	}
//		
//    else if(rc->toggle_switch.SD == REMOTE_ET_SWITCH_VALUE_UP)            //云台底盘跟随
//    {
//        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
//				console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
//				console.platform_cmd = HUMAN_CONTROL_CMD;
//			
//        console.chassis.vx = (rc->Ch4 / RC_ET_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
//        console.chassis.vy = (rc->Ch3 / RC_ET_RESOLUTION * -RC_CHASSIS_MAX_SPEED_Y);
//        console.gimbal.pitch_v = rc->Ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
//        if(rc->toggle_switch.SE == REMOTE_ET_SWITCH_VALUE_DOWN)
//				{
//					console.platform.yaw_v = 0;
//					console.gimbal.yaw_v = -rc->Ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
//				}
//				else if(rc->toggle_switch.SE == REMOTE_ET_SWITCH_VALUE_UP)
//				{
//					console.platform.yaw_v = -rc->Ch1 * ratio;
//					console.gimbal.yaw_v = 0;
//				}
//    }                           
//		
//		  static uint32_t shoot_time = 0;
//		
//		if (console.shoot_cmd == SHOOT_STOP_CMD)
//    {
//        if (rc->toggle_switch.SF == REMOTE_ET_SWITCH_VALUE_UP&&rc->toggle_switch.SG == REMOTE_ET_SWITCH_VALUE_DOWN)
//        {
//            console.shoot_cmd = SHOOT_START_CMD;
//        }	
//    }
//    else if (console.shoot_cmd == SHOOT_START_CMD)
//    {
//        if (rc->toggle_switch.SF == REMOTE_ET_SWITCH_VALUE_DOWN)
//        {
//            console.shoot_cmd = SHOOT_STOP_CMD;
//        }//摩擦轮关闭条件已设置
//				else if (WBUS_switch.switch_state == Rising_edge)
//				{
//              console.shoot.fire_cmd = ONE_FIRE_CMD ;
//				}
//        else if (WBUS_switch.switch_value == REMOTE_ET_SWITCH_VALUE_UP)
//        {
//            shoot_time++;
//            if(shoot_time > 50)
//                console.shoot.fire_cmd = ONE_FIRE_CMD ;
//            else
//                console.shoot.fire_cmd = STOP_FIRE_CMD;
//        }
//				else if (vision_data.can_shoot && console.gimbal_cmd  ==  GIMBAL_VISION_AIM_CMD)
//				{
//					console.shoot.fire_cmd = ONE_FIRE_CMD;
//				}
//        else
//        {
//            console.shoot.fire_cmd = STOP_FIRE_CMD;
//            shoot_time = 0;
//        }
//    }
//}
	
static void Gimbal_RemoteControl_Operation(void)
{ 
    if (console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)           //云台底盘分离      
    {
				console.gimbal_cmd = GIMBAL_VISION_AIM_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
				console.platform_cmd = HUMAN_CONTROL_CMD;

        console.chassis.vx = (console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
        console.chassis.vy = (console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y);
				console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
				console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
    }
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)          //小陀螺
    {
        console.gimbal_cmd = GIMBAL_NORMAL_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;
				console.platform_cmd = PLATFORM_FOLLOW_GIMBAL_CMD;
			
        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * 300;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * 300;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
				console.gimbal.yaw_v = -console.rc->ch1 * RC_GIMBAL_MOVE_RATIO_YAW;
	}
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)            //云台底盘跟随
    {
        console.gimbal_cmd = GIMBAL_FOLLOW_PLATFORM_CMD;
				console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
				console.platform_cmd = HUMAN_CONTROL_CMD;
			
        console.chassis.vx = (console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
        console.chassis.vy = (console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y);
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.platform.yaw_v = -console.rc->ch1 * RC_PLATFORM_MOVE_RATIO_YAW;
    }                           
		
		if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }	
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_STOP_CMD;
        }//摩擦轮关闭条件已设置
//				else if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO2)
//				{
//              console.shoot.fire_cmd = ONE_FIRE_CMD ;
//							shoot_start_time = BSP_GetTime_ms();
//				}
        if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
        {
        
					 console.shoot.fire_cmd = RAPID_FIRE_CMD;
        }
				else if (vision_data.can_shoot && console.gimbal_cmd  ==  GIMBAL_VISION_AIM_CMD)
				{
					console.shoot.fire_cmd = ONE_FIRE_CMD;
				}
        else
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
        }
    }
}

  uint32_t shoot_time = 0;

static void Platform_RemoteControl_Operation(void)
{
	if (console.rc->sw2 == REMOTE_SWITCH_VALUE_CENTRAL)           //云台底盘分离      
    {
				console.gimbal_cmd = GIMBAL_FOLLOW_PLATFORM_CMD;
        console.chassis_cmd = CHASSIS_SEPARATE_GIMBAL_CMD;
				console.platform_cmd = HUMAN_CONTROL_CMD;

        console.chassis.vx = (console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
        console.chassis.vy = (console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y);
				console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
				console.platform.yaw_v = -console.rc->ch1 * RC_PLATFORM_MOVE_RATIO_YAW;
    }
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_DOWN)          //小陀螺
    {
        console.gimbal_cmd = GIMBAL_FOLLOW_PLATFORM_CMD;
        console.chassis_cmd = CHASSIS_SPIN_CMD;
				console.platform_cmd = HUMAN_CONTROL_CMD;
			
        console.chassis.vx = console.rc->ch4 / RC_RESOLUTION * 300;
        console.chassis.vy = console.rc->ch3 / RC_RESOLUTION * 300;
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
				console.platform.yaw_v = -console.rc->ch1 * RC_PLATFORM_MOVE_RATIO_YAW;
	}
		
    else if(console.rc->sw2 == REMOTE_SWITCH_VALUE_UP)            //云台底盘跟随
    {
        console.gimbal_cmd = GIMBAL_FOLLOW_PLATFORM_CMD;
				console.chassis_cmd = CHASSIS_FOLLOW_GIMBAL_CMD;
				console.platform_cmd = HUMAN_CONTROL_CMD;
        console.chassis.vx = (console.rc->ch4 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_X);
        console.chassis.vy = (console.rc->ch3 / RC_RESOLUTION * RC_CHASSIS_MAX_SPEED_Y);
        console.gimbal.pitch_v = console.rc->ch2 * RC_GIMBAL_MOVE_RATIO_PIT;
        console.platform.yaw_v = -console.rc->ch1 * RC_PLATFORM_MOVE_RATIO_YAW;
    }                           
		
		
		
		if (console.shoot_cmd == SHOOT_STOP_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
            console.shoot_cmd = SHOOT_START_CMD;
        }	
    }
    else if (console.shoot_cmd == SHOOT_START_CMD)
    {
        if (wheel_switch.switch_state == REMOTE_SWITCH_CHANGE_3TO1)
        {
					  shoot_time = 0;
            console.shoot_cmd = SHOOT_STOP_CMD;
        }//摩擦轮关闭条件已设置
        else if (wheel_switch.switch_value_raw == REMOTE_SWITCH_VALUE_DOWN)
        {
					shoot_time++;
							if(shoot_time > 30)
						  console.shoot.fire_cmd = ONE_FIRE_CMD ;
				
						if(shoot_time > 100)
						 console.shoot.fire_cmd = RAPID_FIRE_CMD;
        }
				else if (vision_data.can_shoot && console.gimbal_cmd  ==  GIMBAL_VISION_AIM_CMD)
				{
						console.shoot.fire_cmd = RAPID_FIRE_CMD;
				}
        else
        {
            console.shoot.fire_cmd = STOP_FIRE_CMD;
            shoot_time = 0;
        }
    }
}

static void ComputerControl_Operation(void)
{
		uint8_t under_attack = 0;
		fp32 t = 0;
    if(robot_hurt_t.hurt_type == 0x00 && vision_data.state != 1)under_attack = 1;
		if (under_attack == 1)
		{
			t++;
			if(t == 1000)
				under_attack = 0;
		}
		if(under_attack == 1)
			console.gimbal_cmd = GIMBAL_SENTRY_CMD;
		else
			
		console.gimbal_cmd = GIMBAL_SENTRY_CMD;
//		console.gimbal_cmd = GIMBAL_FOLLOW_PLATFORM_CMD;
    console.chassis_cmd = CHASSIS_NAV_CMD;
		console.shoot_cmd = SHOOT_START_CMD;
		console.platform_cmd = COMPUTER_CONTROL_CMD;
		
		if (vision_data.can_shoot && console.gimbal_cmd  ==  GIMBAL_SENTRY_CMD)
		{
			console.shoot.fire_cmd = ONE_FIRE_CMD;
		}
		else
    {
      console.shoot.fire_cmd = STOP_FIRE_CMD;
    }
}

Console_t* Console_Pointer(void)
{
    return &console;
}
