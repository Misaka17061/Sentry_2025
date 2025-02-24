/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-01-09 19:05:07
 * @LastEditors: your name
 * @LastEditTime: 2025-01-25 14:47:35
 * @FilePath: \MDK-ARMd:\RM_work\ZJ_Hero_Group_Work\Hero2025_V1.50\Application\ui_task.c
 * @Description: 
 * 
 * Copyright (c) 2025 by ${git_name_email}, All Rights Reserved. 
 */
/* 包含头文件 ----------------------------------------------------------------*/
#include "ui_task.h"
#include "referee_system.h"
#include "client_ui_base.h"
#include "user_protocol.h"
#include "cmsis_os.h"
#include "bsp_init.h"
/* 私有类型定义 --------------------------------------------------------------*/
osThreadId UiTaskHandle;
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
UiGraphicData_t xx1;      //加线①
UiGraphicData_t xx2;
UiGraphicData_t xx3;
UiGraphicData_t xx4;
UiGraphicData_t xx5;
UiGraphicData_t xx6;
UiGraphicData_t xx7;
UiGraphicData_t xx8;
UiGraphicData_t xx9;
UiGraphicData_t xx10;

UiGraphicData_t xx7;
UiGraphicData_t cx1;
UiGraphicData_t cx2;
UiGraphicData_t cx3;
UiGraphicData_t cx4;
UiGraphicData_t cx5;//瞄准线
UiGraphicData_t cx6;
UiGraphicData_t cx7;
UiGraphicData_t cx8;
UiGraphicData_t cx9;
UiGraphicData_t cx10;
UiGraphicData_t cx11;
UiGraphicData_t cx12;

UiStringData_t qz;
UiGraphicData_t q;//小陀螺
UiGraphicData_t h;
UiStringData_t hz;//底盘
UiGraphicData_t mc;
UiStringData_t mcz;//自瞄
UiStringData_t sp;
UiGraphicData_t s;//超电开启关闭
UiNumberData_t cd;//超电
UiNumberData_t pit;//超电
UiNumberData_t inf;
//x轴-490——-390，y轴+-60
//x轴-535——-960，y轴-30——540不能画
UiStringData_t xtl;
UiStringData_t lxy;
UiStringData_t czk;
UiStringData_t chz;
UiGraphicData_t dc;
/*****2025赛季***********/
UiStringData_t gimbal_mode;
UiStringData_t chassis_mode;
UiStringData_t shoot_magazine_mode;
UiStringData_t shoot_fri_mode;
UiStringData_t vision_mode;
UiStringData_t supercap_mode;
UiStringData_t heat_limit;

/* 扩展变量 ------------------------------------------------------------------*/
extern ChassisHandle_t chassis_handle;
/* 私有函数原形 --------------------------------------------------------------*/
static TransmitHandle_t client_ui_tx_handle;
static uint8_t client_ui_tx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];
static int32_t ClientUI_Data(void);
static void ClientUI_UploadDataHook(uint8_t *data, uint16_t len);
static void Gimbal_CtrlMode_UI(Console_t* console);	//云台控制模式UI
static void Chassis_CtrlMode_UI(Console_t* console);	//底盘控制模式UI
static void ShootMagazine_Mode_UI(Console_t* console); //弹舱盖模式UI
static void ShootFriMode_Mode_UI(Console_t* console); //摩擦轮模式UI
static void Vision_Mode_UI(Console_t* console); //视觉模式UI
static void SuperCap_Mode_UI(Console_t* console); //超级电容模式UI
static void Heat_Limit_UI(Console_t* console); //加热限制UI
/* 函数体 --------------------------------------------------------------------*/

void UiTask(void const *argument)
{
	for(;;)
  {
  	ClientUI_Data();
  	osDelay(50);
  }
}


void UiTask_Init(void)
{

  Comm_TransmitInit(&client_ui_tx_handle, client_ui_tx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, ClientUI_UploadDataHook);
  ClientUI_Init(&client_ui_tx_handle);
  osThreadDef(Ui_task, UiTask, osPriorityNormal, 0, 256);            //比正常线程等级要低一个
  UiTaskHandle = osThreadCreate(osThread(Ui_task), NULL);            //控制任务开始

}

static void ClientUI_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_UART_TransmitData(&com1_obj, data, len);
}

    // SoftwareTimerRegister(ClientUI_Data, (void*)NULL, 100);
    
static int32_t ClientUI_Data(void)
{
   Comm_GimbalInfo_t * gimbal_info = GimbalInfo_Pointer();
    uint16_t robot_id = RefereeSystem_GetRobotID();
    ClientUI_SetHeaderSenderID(robot_id);          //发送者ID为获取的机器人ID
    ClientUI_SetHeaderReceiverID(0x100 + robot_id); //接收者ID为选手端ID，为0x100+机器人ID
//	ClientUI_DrawString(&lxy, "lxy", 0,540, 65, "ok", 15, 3, UI_COLOR_YELLOW);
//初始化模式
    
	if(chassis_handle.console->gimbal_cmd==GIMBAL_INIT_CMD || chassis_handle.console->chassis_cmd==CHASSIS_RELEASE_CMD)
	{
		ClientUI_DrawString(&xtl, "xtl", 0,540, 35, "mode shoot mag gim", 15, 3, UI_COLOR_YELLOW);
	}
//UI:热量限制开启	
//	else if(chassis_handle.ctrl_mode == CHASSIS_RELAX && 
//	   chassis_handle.console->shoot_cmd==SHOOT_RELEASE_CMD &&
//	   chassis_handle.console->gimbal_cmd==GIMBAL_RELEASE_CMD &&
//	   chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "rex rex init rex ON");//relax     relax        init        AIM    ON
//		                                                   //底盘模式  发射机构模式  弹舱盖模式   自瞄   热量限制 
//	}
////UI:热量限制关闭		
//	else if(chassis_handle.ctrl_mode == CHASSIS_RELAX && 
//	   chassis_handle.console->shoot_cmd==SHOOT_RELEASE_CMD &&
//	   chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	   chassis_handle.console->gimbal_cmd==GIMBAL_RELEASE_CMD &&
//	   chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "rex rex init rex OFF");
//	}
////	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd == SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd == MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "fol shoot init nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd == SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd == MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "fol shoot init nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd == SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd == MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "fol shoot open nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd == SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd == MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	       chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "fol shoot open nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "fol shoot close nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "fol shoot close nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "fol stop init nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "fol stop init nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "fol stop open nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "fol stop open nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "fol stop close nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_FOLLOW_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "fol stop close nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "sep shoot init nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "sep shoot init nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "sep shoot open nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "sep shoot open nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "sep shoot close nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "sep shoot close nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "sep stop init nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "sep stop init nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "sep stop open nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "sep stop open nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "sep stop close nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SEPARATE_GIMBAL &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "sep stop close nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "spin shoot m_init nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "spin shoot m_init nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "spin shoot open nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "spin shoot open nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "spin shoot close nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_START_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "spin shoot close nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "spin stop m_init nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_INIT_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "spin stop m_init nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "spin stop open nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_ON_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "spin stop open nor OFF");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_on)
//	{
//		ClientUI_UpdateString(&xtl, "spin stop close nor ON");
//	}
//	
//	else if(chassis_handle.ctrl_mode == CHASSIS_SPIN &&
//		    chassis_handle.console->shoot_cmd==SHOOT_STOP_CMD &&
//	        chassis_handle.console->magazine_cmd==MAGAZINE_OFF_CMD &&
//	        chassis_handle.console->gimbal_cmd==GIMBAL_NORMAL_CMD &&
//	        chassis_handle.console->heat_limit==Heat_limit_off)
//	{
//		ClientUI_UpdateString(&xtl, "spin stop close nor OFF");
//	}
	
	ClientUI_Update(&xtl, NULL, NULL, NULL, NULL, NULL, NULL);
}


static void Gimbal_CtrlMode_UI(Console_t* console)	//云台控制模式UI
{
        switch (console->gimbal_cmd)
        {
            case GIMBAL_INIT_CMD:               
            {   //对接南航UI设计接口函数
                ClientUI_DrawString(&gimbal_mode, "gimbal_mode", 0,240, 700, "INIT", 15, 3, UI_COLOR_YELLOW);
            }break;

            case GIMBAL_FOLLOW_PLATFORM_CMD:           //云台分离模式
            {
                ClientUI_UpdateString(&gimbal_mode, "FOLLOW");//relax     relax        init        AIM    ON
		                                                  		  //底盘模式  发射机构模式  弹舱盖模式   自瞄   热量限制
            }break;

            case GIMBAL_NORMAL_CMD:             //云台跟随模式
            {
                ClientUI_UpdateString(&gimbal_mode, "NORMAL");//relax     relax        init        AIM    ON
		                                                  		  //底盘模式  发射机构模式  弹舱盖模式   自瞄   热量限制
            }break;
            case GIMBAL_VISION_AIM_CMD:         //云台视觉跟踪模式
            {
                ClientUI_UpdateString(&gimbal_mode, "VISION");//relax     relax        init        AIM    ON
		                                                  		  //底盘模式  发射机构模式  弹舱盖模式   自瞄   热量限制
            }break;
            default:
                break;
        }
}

static void Chassis_CtrlMode_UI(Console_t* console)	//底盘控制模式UI
{


}


static void ShootMagazine_Mode_UI(Console_t* console) //弹舱盖模式UI
{

}

static void ShootFriMode_Mode_UI(Console_t* console) //摩擦轮模式UI
{

}

static void Vision_Mode_UI(Console_t* console) //视觉模式UI
{

}

static void SuperCap_Mode_UI(Console_t* console) //超级电容模式UI
{

}

static void Heat_Limit_UI(Console_t* console) //加热限制UI
{

}