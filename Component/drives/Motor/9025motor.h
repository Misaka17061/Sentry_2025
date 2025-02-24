#ifndef MOTOR_9025_H
#define MOTOR_9025_H

/* 包含头文件 ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "bsp_can.h"
#include "pid.h"
#include "motor.h"
/* 类型定义 ------------------------------------------------------------------*/
typedef struct
{
    uint8_t command_byte;				//命令字节
		int8_t temperature;        //电机温度
		int16_t power;							//转矩电流
    int16_t speed;							//电机速度
    uint16_t encoder;						//编码器位置
		uint16_t encoderRaw;        //编码器原始位置
		uint16_t encoderOffset;			//编码器零偏
		pid_t    anglePid;					//角度环PID
		pid_t    speedPid;					//速度环PID
		pid_t    iqPid;							//转矩环PID
		uint32_t circleAngle;				//电机单圈角度
} MotorInfo_9025_t;

typedef struct
{
    MotorInfo_9025_t*	motor_info;
		uint16_t 				encoderOffset;
		int16_t 				powerControl;
		int32_t 				speedControl;
		int32_t 				angleControl;
		uint16_t 				maxSpeed;
		uint8_t 				spinDirection;
		Sensor_t  			sensor;
		fp32            given_value;
		Double_PID_t    dpid;
		MotorMode_e 		mode;
	  MotorMode_e 		last_mode;
} Motor_9025_t;


/* 宏定义 --------------------------------------------------------------------*/
#define M9025_MOTOR_MAX_POWER     (850.0f)
#define M9025_MOTOR_MAX_SPEED     (120.0f)

#define MOTOR_9025_ENCODER_RANGE         (32767)
#define MOTOR_9025_ENCODER_RANGE_HALF    (16383.5)
#define ENCODER_9025_ANGLE_RATIO         (32767.0f / 360.0f)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/
void Motor_9025_DataParse(MotorInfo_9025_t *ptr, uint8_t data[]);
void Motor_Off(CAN_Object_t* obj, uint32_t std_id);
void Motor_On(CAN_Object_t* obj, uint32_t std_id);
void Torque_Control(CAN_Object_t* obj, uint32_t std_id, int16_t powerControl);
void Speed_Control(CAN_Object_t* obj, uint32_t std_id, int32_t speedControl);
void Multi_Loop_Angle_Control1(CAN_Object_t* obj, uint32_t std_id, int32_t angleControl);
void Multi_Loop_Angle_Control2(CAN_Object_t* obj, uint32_t std_id, uint16_t maxSpeed,int32_t angleControl);
void Single_Loop_Angle_Control1(CAN_Object_t* obj, uint32_t std_id, uint8_t spinDirection,uint32_t angleControl);
void Single_Loop_Angle_Control2(CAN_Object_t* obj, uint32_t std_id, uint8_t spinDirection, uint16_t maxSpeed,uint32_t angleControl);
void Increment_Angle_Control1(CAN_Object_t* obj, uint32_t std_id,int32_t angleControl);
void Increment_Angle_Control2(CAN_Object_t* obj, uint32_t std_id, uint16_t maxSpeed,int32_t angleControl);
void PID_Read(CAN_Object_t* obj, uint32_t std_id);
void PID_RAMset(CAN_Object_t* obj, uint32_t std_id, fp32 anglePidKp, fp32 anglePidKi, fp32 speedPidKp, fp32 speedPidKi, fp32 iqPidKp, fp32 iqPidKi);
void PID_ROMset(CAN_Object_t* obj, uint32_t std_id, fp32 anglePidKp, fp32 anglePidKi, fp32 speedPidKp, fp32 speedPidKi, fp32 iqPidKp, fp32 iqPidKi);
void Encoder_Read(CAN_Object_t* obj, uint32_t std_id);
void Encoder_Offset(CAN_Object_t* obj, uint32_t std_id,  uint16_t encoderOffset);
void Angle_Read(CAN_Object_t* obj, uint32_t std_id);
int16_t Motor_9025_RelativePosition(int16_t ecd, int16_t offset);
MotorInfo_9025_t* PlatformMotor_Pointer(void);
#endif  // MOTOR_9025_H
