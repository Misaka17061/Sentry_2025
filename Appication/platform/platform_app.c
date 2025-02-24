/* 包含头文件 ----------------------------------------------------------------*/
#include "platform_app.h"
#include "sentry_def.h"
#include "app_init.h"
#include "cmsis_os.h"
#include "comm_protocol.h"
#include "referee_system.h"
#include "user_protocol.h"
#include "computer_protocol.h"

#include "timer_task.h"
#include "detect_task.h"
#include "td.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
// 上位机收发
static TransmitHandle_t nav_tx_handle;
static uint8_t nav_tx_fifo_buffer[COMPUTER_DATA_FIFO_SIZE];

static ReceiveHandle_t nav_rx_handle;
static uint8_t nav_rx_fifo_buffer[COMPUTER_DATA_FIFO_SIZE];

// 裁判系统接收
static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

// 用户协议收发
static TransmitHandle_t platform_tx_handle;
static uint8_t platform_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static ReceiveHandle_t platform_rx_handle;
static uint8_t platform_rx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

/* 扩展变量 ------------------------------------------------------------------*/
PlatformHandle_t platform_handle;

TD_conctrol td;
fp32 ecd_ratio;
/* 私有函数原形 --------------------------------------------------------------*/
static void COM1_UploadDataHook(uint8_t *data, uint16_t len);
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);

static int32_t Nav_DecisionInfoUploadCallback(void *argc);
static int32_t PlatformInfoUploadCallback(void *argc);

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);

 
/* 函数体 --------------------------------------------------------------------*/
void PlatformAppConfig(void)
{
	platform_handle.yaw_motor.motor_info = PlatformMotor_Pointer();
		
	platform_handle.console = Console_Pointer();
	platform_handle.imu  = IMU_GetDataPointer();
    platform_handle.platform_can  = &can1_obj;
	platform_handle.ctrl_mode = PLATFORM_INIT;
	
	platform_handle.yaw_motor.encoderOffset = 27487;
	ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_B_REDUCTION_RATIO / ENCODER_9025_ANGLE_RATIO;
		
	pid_init(&platform_handle.yaw_motor.dpid.outer_pid, POSITION_PID, 850.0f, 0.0f,
             16.0f, 0.0f, 3.0f);/*调速*//*慢加P，抖加D*/ /*8,0,1*/
    pid_init(&platform_handle.yaw_motor.dpid.inter_pid, POSITION_PID, 850, 0.0f,
             7.08f, 0.0f, 0.0f);/*调力*/ /*15,0,1*/
	
	pid_init(&platform_handle.follow_pid.inter_pid, POSITION_PID, 2.0f, 20.0f,
             0.03f, 0.0f, 0.05f);
		pid_init(&platform_handle.follow_pid.outer_pid, POSITION_PID, 2.0f, 20.0f,
             0.0f, 0.0f, 0.0f);

		
    /*--------------------event-----------------|-----------enable-----------|-offline time-|-beep_times-*/
    OfflineHandle_Init(OFFLINE_PLATFORM_YAW,            OFFLINE_ERROR_LEVEL,       100,         0);
//    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         0);
	OfflineHandle_Init(OFFLINE_GIMBAL_INFO,           	OFFLINE_ERROR_LEVEL,       100,         1);
	OfflineHandle_Init(OFFLINE_NAV_INFO,           			OFFLINE_ERROR_LEVEL,       100,         1);
//	  OfflineHandle_Init(OFFLINE_WBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);
	OfflineHandle_Init(OFFLINE_DBUS,            			OFFLINE_ERROR_LEVEL,     		100,         1);
	
	Comm_TransmitInit(&nav_tx_handle, nav_tx_fifo_buffer, COMPUTER_DATA_FIFO_SIZE, COM1_UploadDataHook);
    Comm_ReceiveInit(&nav_rx_handle, COMPUTER_PROTOCOL_HEADER_SOF, nav_rx_fifo_buffer, COMPUTER_DATA_FIFO_SIZE, ComputerProtocol_ParseHandler);
	SoftwareTimerRegister(Nav_DecisionInfoUploadCallback, (void*)NULL, 5);
	
	Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
	
	Comm_TransmitInit(&platform_tx_handle, platform_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
    Comm_ReceiveInit(&platform_rx_handle, USER_PROTOCOL_HEADER_SOF, platform_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
	SoftwareTimerRegister(PlatformInfoUploadCallback, (void*)NULL, 20);
	
	
	BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
	BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
	BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
	BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
	BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

// 发送勾子函数（用于初始化发送句柄）
static void COM1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_UART_TransmitData(&com1_obj, data, len);
}

static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, PLATFORM_DATA_STD_ID, data, len);
}

// 软件时间上传回调函数
static int32_t Nav_DecisionInfoUploadCallback(void *argc)
{
	Comm_DecisionInfo_t* info = DecisionInfo_Pointer();
	uint16_t robot_id = RefereeSystem_GetRobotID();
	info->game_state = RefereeSystem_GameState_Pointer();
	info->game_robot_HP_t = RefereeSystem_GameRobot_HP_t_Pointer();
	info->bullet_remaining_t = RefereeSystem_GameRobot_bullet_remaining_t_Pointer();
	info->x = 0.0f;
//	info->bullet_remaining_num_17mm = RefereeSystem_GameRobot_bullet_remaining_t_Pointer()->bullet_remaining_num_17mm;
//	if (robot_id > 100)     //ID大于100是蓝方  应该打红方；
//    {
//      info->ally_sentry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->blue_7_robot_HP;
//		info->ally_infantry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->blue_3_robot_HP 
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->blue_4_robot_HP
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->blue_5_robot_HP;
//		info->ally_hero_HP = RefereeSystem_GameRobot_HP_t_Pointer()->blue_1_robot_HP;
//		info->enemy_sentry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->red_7_robot_HP;
//		info->enemy_infantry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->red_3_robot_HP 
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->red_4_robot_HP
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->red_5_robot_HP;
//		info->enemy_hero_HP = RefereeSystem_GameRobot_HP_t_Pointer()->red_1_robot_HP;
//    }
//    else if (robot_id > 1)
//    {
//		info->ally_sentry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->red_7_robot_HP;
//		info->ally_infantry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->red_3_robot_HP 
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->red_4_robot_HP
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->red_5_robot_HP;
//		info->ally_hero_HP = RefereeSystem_GameRobot_HP_t_Pointer()->red_1_robot_HP;
//		info->enemy_sentry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->blue_7_robot_HP;
//		info->enemy_hero_HP = RefereeSystem_GameRobot_HP_t_Pointer()->blue_3_robot_HP 
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->blue_4_robot_HP
//								 + RefereeSystem_GameRobot_HP_t_Pointer()->blue_5_robot_HP;
//		info->enemy_infantry_HP = RefereeSystem_GameRobot_HP_t_Pointer()->blue_1_robot_HP;
//    }
	Comm_TransmitData_Computer(&nav_tx_handle, (uint8_t*)info, sizeof(Comm_DecisionInfo_t));
	
	return 0;
}

static int32_t PlatformInfoUploadCallback(void *argc)
{
	Comm_NavCmd_t* NavCmd_info = NavCmd_Pointer();
	NavCmd_info->vx = FourBytesToFloat(NavInfo_Pointer()->vx);
	NavCmd_info->vy = FourBytesToFloat(NavInfo_Pointer()->vy);
	NavCmd_info->vw = FourBytesToFloat(NavInfo_Pointer()->vw);
	Comm_TransmitData(&platform_tx_handle, USER_PROTOCOL_HEADER_SOF, NAV_CMD_ID, (uint8_t*)NavCmd_info, sizeof(Comm_NavCmd_t));
	Comm_PlatformInfo_t* Platform_info = PlatformInfo_Pointer();
	Platform_info->mode = platform_handle.ctrl_mode; 
	Platform_info->yaw_ecd_angle = platform_handle.yaw_motor.sensor.relative_angle; 
	Platform_info->gyro_angle = platform_handle.yaw_motor.sensor.gyro_angle;
	Comm_TransmitData(&platform_tx_handle, USER_PROTOCOL_HEADER_SOF, PLATFORM_INFO_CMD_ID, (uint8_t*)Platform_info, sizeof(Comm_PlatformInfo_t));
	
	return 0;
}


// 接收回调
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&platform_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{
	Comm_ReceiveData(&nav_rx_handle, data, len);
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{

}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{

    switch (std_id)
    {
        case PLATFORM_MOTOR_YAW_MESSAGE_ID:
        {
            Motor_9025_DataParse(platform_handle.yaw_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_YAW);
        }break;
		case CHASSIS_DATA_STD_ID:
		{
			Comm_ReceiveData(&platform_rx_handle, data, dlc);
		}break;
		case REFEREE_DATA_STD_ID:
		{
			Comm_ReceiveData(&referee_rx_handle, data, dlc);
		}break;
		case GIMBAL_DATA_STD_ID:
		{
			Comm_ReceiveData(&platform_rx_handle, data, dlc);
		}break;
        default:
            break;
    }
}

static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{

}

 
