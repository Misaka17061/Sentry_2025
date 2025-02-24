#include "gimbal_app.h"
#include "sentry_def.h"
#include "app_init.h"

#include "comm_protocol.h"
#include "referee_system.h"
#include "user_protocol.h"

#include "timer_task.h"
#include "detect_task.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
static TransmitHandle_t vision_tx_handle;
static uint8_t vision_tx_fifo_buffer[COMPUTER_DATA_FIFO_SIZE];

static ReceiveHandle_t vision_rx_handle;
static uint8_t vision_rx_fifo_buffer[COMPUTER_DATA_FIFO_SIZE];

static ReceiveHandle_t referee_rx_handle;
static uint8_t referee_rx_fifo_buffer[REFEREE_SYSTEM_FIFO_SIZE];

static TransmitHandle_t gimbal_tx_handle;
static uint8_t gimbal_tx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

static ReceiveHandle_t gimbal_rx_handle;
static uint8_t gimbal_rx_fifo_buffer[GIMBAL_CHASSIS_DATA_FIFO_SIZE];

/* 扩展变量 ------------------------------------------------------------------*/
GimbalHandle_t gimbal_handle;
pid_t yaw_vision_pid;
pid_t pitch_vision_pid;
float attitude_yaw_initial;
AutoAim_t yaw_aim;
/* 私有函数原形 --------------------------------------------------------------*/
static void COM1_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t Vision_RobotInfoUploadCallback(void *argc);
static void CAN1_UploadDataHook(uint8_t *data, uint16_t len);
static int32_t GimbalInfoUploadCallback(void *argc);
static void COM1_ReceiveCallback(uint8_t* data, uint16_t len);
static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len);
static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc);
static void COM2_ReceiveCallback(uint8_t* data, uint16_t len);
static void Aim_init(AutoAim_t* Aim);

/* 函数体 --------------------------------------------------------------------*/
void GimbalAppConfig(void)
{
	
		gimbal_handle.yaw_motor.motor_info = GimbalMotorYaw_Pointer();
		gimbal_handle.pitch_motor.motor_info = GimbalMotorPitch_Pointer();
	
		gimbal_handle.console = Console_Pointer();
    gimbal_handle.imu  = IMU_GetDataPointer();
    gimbal_handle.gimbal_can  = &can1_obj;
		gimbal_handle.ctrl_mode = GIMBAL_INIT;
	
		gimbal_handle.yaw_motor.offset_ecd = 4620;
    gimbal_handle.pitch_motor.offset_ecd = 5430;
	
		gimbal_handle.yaw_motor.ecd_ratio = YAW_MOTO_POSITIVE_DIR * YAW_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.pitch_motor.ecd_ratio = PITCH_MOTO_POSITIVE_DIR * PITCH_REDUCTION_RATIO / ENCODER_ANGLE_RATIO;
    gimbal_handle.yaw_motor.max_relative_angle = 70;
    gimbal_handle.yaw_motor.min_relative_angle = -70;
    gimbal_handle.pitch_motor.max_relative_angle = 30;
    gimbal_handle.pitch_motor.min_relative_angle = -20;
	
		attitude_yaw_initial = gimbal_handle.imu->attitude.yaw;
		Aim_init(&yaw_aim);
	
    pid_init(&gimbal_handle.yaw_motor.dpid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             14.0f, 0.0f, 3.0f);/*调速*//*慢加P，抖加D*/ /*60,0,40*/
    pid_init(&gimbal_handle.yaw_motor.dpid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 3000.0f,
             200.0f, 20.0f, 30.0f);/*调力*//**/
    pid_init(&gimbal_handle.pitch_motor.dpid.outer_pid, POSITION_PID, 2000.0f, 0.0f,
             60.0f, 0.0f, 10.0f);
    pid_init(&gimbal_handle.pitch_motor.dpid.inter_pid, POSITION_PID, GM6020_MOTOR_MAX_CURRENT, 6000.0f,
             25.0f, 0.01f, 10.0f);
		
		pid_init(&gimbal_handle.follow_pid, POSITION_PID, 300.0f, 50.0f,
             0.5f, 0.0f, 1.0f);
		
		pid_init(&yaw_vision_pid,POSITION_PID, 200.0f, 200.0f,0.019, 0, 0.1); 	//d0.4
		pid_init(&pitch_vision_pid, POSITION_PID, 200.0f, 200.0f,0.011, 0, 0.11); 
		
    /*--------------------event-----------------|-----------enable-----------|-offline time-|-beep_times-*/
    OfflineHandle_Init(OFFLINE_VISION_INFO,             OFFLINE_WARNING_LEVEL,     200,         0);
    OfflineHandle_Init(OFFLINE_GIMBAL_PITCH,            OFFLINE_ERROR_LEVEL,       100,         0);
    OfflineHandle_Init(OFFLINE_GIMBAL_YAW,              OFFLINE_ERROR_LEVEL,       100,         0);
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR1,   OFFLINE_ERROR_LEVEL,       100,         0);
    OfflineHandle_Init(OFFLINE_FRICTION_WHEEL_MOTOR2,   OFFLINE_ERROR_LEVEL,       100,         0);
    OfflineHandle_Init(OFFLINE_TRIGGER_MOTOR,           OFFLINE_ERROR_LEVEL,       100,         0);
    OfflineHandle_Init(OFFLINE_REFEREE_SYSTEM,          OFFLINE_WARNING_LEVEL,     100,         0);
	  OfflineHandle_Init(OFFLINE_PLATFORM_INFO,           OFFLINE_ERROR_LEVEL,       100,         2);
//	  OfflineHandle_Init(OFFLINE_WBUS,                    OFFLINE_WARNING_LEVEL,     100,         0);
		OfflineHandle_Init(OFFLINE_DBUS,            			OFFLINE_ERROR_LEVEL,     		100,         1);
	
    Comm_TransmitInit(&vision_tx_handle, vision_tx_fifo_buffer, COMPUTER_DATA_FIFO_SIZE, COM1_UploadDataHook);
		SoftwareTimerRegister(Vision_RobotInfoUploadCallback, (void*)NULL, 5);
    Comm_ReceiveInit(&vision_rx_handle, COMPUTER_PROTOCOL_HEADER_SOF, vision_rx_fifo_buffer, COMPUTER_DATA_FIFO_SIZE, ComputerProtocol_ParseHandler);
		
		Comm_ReceiveInit(&referee_rx_handle, REFEREE_SYSTEM_HEADER_SOF, referee_rx_fifo_buffer, REFEREE_SYSTEM_FIFO_SIZE, RefereeSystem_ParseHandler);
		
    Comm_TransmitInit(&gimbal_tx_handle, gimbal_tx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, CAN1_UploadDataHook);
		SoftwareTimerRegister(GimbalInfoUploadCallback, (void*)NULL, 10);
		Comm_ReceiveInit(&gimbal_rx_handle, USER_PROTOCOL_HEADER_SOF, gimbal_rx_fifo_buffer, GIMBAL_CHASSIS_DATA_FIFO_SIZE, UserProtocol_ParseHandler);
		
		BSP_UART_SetRxCallback(&dbus_obj, DBUS_ReceiveCallback);
		BSP_UART_SetRxCallback(&com1_obj, COM1_ReceiveCallback);
		BSP_UART_SetRxCallback(&com2_obj, COM2_ReceiveCallback);
	  BSP_CAN_SetRxCallback(&can1_obj, CAN1_ReceiveCallback);
		BSP_CAN_SetRxCallback(&can2_obj, CAN2_ReceiveCallback);
}

static void COM1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_UART_TransmitData(&com1_obj, data, len);
}

static void CAN1_UploadDataHook(uint8_t *data, uint16_t len)
{
    BSP_CAN_TransmitData(&can1_obj, GIMBAL_DATA_STD_ID, data, len);
}

static int32_t Vision_RobotInfoUploadCallback(void *argc)
{
    Comm_RobotInfo_t* info = RobotInfo_Pointer();
    uint16_t robot_id = RefereeSystem_GetRobotID();
    info->data_head = 0xAA;
    if (robot_id > 100)     //ID大于100是蓝方  应该打红方；
    {
       info->enemy_color = Red;
    }
    else if (robot_id > 1)
    {
			 info->enemy_color = Blue;
    }
    else
    {
       info->enemy_color = AllColor;
    }
    info->speed = 0;
    info->yaw_relative_angle = (gimbal_handle.imu->attitude.yaw - attitude_yaw_initial) * ANGLE_TO_RAD;
    info->pitch_relative_angle = (gimbal_handle.imu->attitude.pitch - gimbal_handle.imu->i_attitude.i_pitch) * ANGLE_TO_RAD;
    info->bullet_speed = 0;		
		info->data_tail = 0xA5;
    Comm_TransmitData_Computer(&vision_tx_handle, (uint8_t*)info, sizeof(Comm_RobotInfo_t));
    return 0;
}

static int32_t GimbalInfoUploadCallback(void *argc)
{
    Comm_GimbalInfo_t* info = GimbalInfo_Pointer();
    info->mode = gimbal_handle.ctrl_mode;
		info->yaw_ecd_angle = gimbal_handle.yaw_motor.sensor.relative_angle;
	
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, GIMBAL_INFO_CMD_ID, (uint8_t*)info, sizeof(Comm_GimbalInfo_t));
    return 0;
}

static void COM1_ReceiveCallback(uint8_t* data, uint16_t len)
{     
   Comm_ReceiveData(&vision_rx_handle, data, len);
}

static void DBUS_ReceiveCallback(uint8_t* data, uint16_t len)
{
    RC_DataParser(RC_GetDataPointer(), data, len);
    Comm_TransmitData(&gimbal_tx_handle, USER_PROTOCOL_HEADER_SOF, RC_DATA_CMD_ID, data, len);
    OfflineHandle_TimeUpdate(OFFLINE_DBUS);
}

static void CAN1_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case GIMBAL_MOTOR_YAW_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.yaw_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_YAW);
        }break;
        case GIMBAL_MOTOR_PITCH_MESSAGE_ID:
        {
            Motor_DataParse(gimbal_handle.pitch_motor.motor_info, data);
            OfflineHandle_TimeUpdate(OFFLINE_GIMBAL_PITCH);
        }break;
				case CHASSIS_DATA_STD_ID:
				{
						Comm_ReceiveData(&gimbal_rx_handle, data, dlc);
				}break;
				case PLATFORM_DATA_STD_ID:
				{
						Comm_ReceiveData(&gimbal_rx_handle, data, dlc);
				}break;
				case REFEREE_DATA_STD_ID:
				{
						Comm_ReceiveData(&referee_rx_handle, data, dlc);
				}break;
        default:
            break;
    }
}

static void COM2_ReceiveCallback(uint8_t* data, uint16_t len)
{
		Comm_ReceiveData(&vision_rx_handle, data, len);
}


static void CAN2_ReceiveCallback(uint32_t std_id, uint8_t *data, uint32_t dlc)
{
    switch (std_id)
    {
        case FRICTION_WHEEL_1_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_1_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR1);
        }break;
        case FRICTION_WHEEL_2_MESSAGE_ID:
        {
            Motor_DataParse(FrictionWheelMotor_2_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_FRICTION_WHEEL_MOTOR2);
        }break;
        case TRIGGER_MOTOR_MESSAGE_ID:
        {
            Motor_DataParse(TriggerMotor_Pointer(), data);
            OfflineHandle_TimeUpdate(OFFLINE_TRIGGER_MOTOR);
        }break;
        default:
            break;
    }
}

static void Aim_init(AutoAim_t* Aim)
{
	Aim->aim_mode = RELAX;
	Aim->pid_SAtime = 0;
  Aim->aiming_time = 0;
  Aim->stay_time = 0;
  Aim->systeam_time = 0;
  Aim->tol_angle = 2.0f;  //人为规定消抖角度范围(-tol_angle,tol_angle)
  Aim->tol_time = 700;//the aiming-done continueous time that user set 人为规定
  Aim->Sstart_time = Aim->Astart_time = 0;//消抖持续时间,单次自瞄持续时间
  Aim->first_aim = 0;//初入消抖范围标志
  Aim->aim_flag = 0; //新的单次自瞄标志
	Aim->Ap_parm = 3500;//Sp_parm = 5000;//user set
  Aim->Sp_parm = 0.0085;
  Aim->enable_paramSA = 1;
}
