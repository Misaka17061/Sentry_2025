/* 包含头文件 ----------------------------------------------------------------*/
#include "9025motor.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
MotorInfo_9025_t platform_motor;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
/*************************************************
 * Function: Motor_9025_DataParse
 * Description: 9025电机回调数据处理
 * Input: ptr 电机信息指针
 *        data 数据指针
 * Return: 无
*************************************************/
void Motor_9025_DataParse(MotorInfo_9025_t *ptr, uint8_t data[])
{
    ptr->command_byte = data[0];
		switch(ptr->command_byte)
		{
			case 0xA0:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA2:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA3:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA4:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA5:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA6:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA7:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0xA8:
			{
				ptr->temperature = (int8_t)data[1];
				ptr->power = (int16_t)((data[3] << 8) | data[2]);
				ptr->speed = (int16_t)((data[5] << 8) | data[4]);
				ptr->encoder = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0x30:
			{
				ptr->anglePid.p = data[2];
				ptr->anglePid.i = data[3];
				ptr->speedPid.p = data[4];
				ptr->speedPid.i = data[5];
				ptr->iqPid.p = data[6];
				ptr->iqPid.i = data[7];
			}break;
			case 0x31:
			{
				ptr->anglePid.p = data[2];
				ptr->anglePid.i = data[3];
				ptr->speedPid.p = data[4];
				ptr->speedPid.i = data[5];
				ptr->iqPid.p = data[6];
				ptr->iqPid.i = data[7];
			}break;
			case 0x32:
			{
				ptr->anglePid.p = data[2];
				ptr->anglePid.i = data[3];
				ptr->speedPid.p = data[4];
				ptr->speedPid.i = data[5];
				ptr->iqPid.p = data[6];
				ptr->iqPid.i = data[7];
			}break;
			case 0x90:
			{
				ptr->encoder = (uint16_t)((data[3] << 8) | data[2]) & 0x7FFF;
				ptr->encoderRaw = (uint16_t)((data[5] << 8) | data[4]) & 0x7FFF;
				ptr->encoderOffset = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0x91:
			{
				ptr->encoderOffset = (uint16_t)((data[7] << 8) | data[6]) & 0x7FFF;
			}break;
			case 0x94:
			{
				ptr->circleAngle = (uint32_t)(data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4]);
			}break;
		}
		
}

/*************************************************
 * Function: Motor_Off
 * Description: 电机关闭数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 * Return: 无
*************************************************/
void Motor_Off(CAN_Object_t* obj, uint32_t std_id)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x80;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Motor_On
 * Description: 电机运行数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 * Return: 无
*************************************************/
void Motor_On(CAN_Object_t* obj, uint32_t std_id)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x88;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Torque_Control
 * Description: 电机开环控制数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        powerControl 电机电流值，数值范围-850~850
 * Return: 无
*************************************************/
void Torque_Control(CAN_Object_t* obj, uint32_t std_id, int16_t powerControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA0;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = (uint8_t)powerControl;
    TxData[5] = (uint8_t)(powerControl >> 8);
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Speed_Control
 * Description: 电机速度控制数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        speedControl 电机速度值，对应实际转速为0.01dps/LSB
 * Return: 无
*************************************************/
void Speed_Control(CAN_Object_t* obj, uint32_t std_id, int32_t speedControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA2;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = (uint8_t)speedControl;
    TxData[5] = (uint8_t)(speedControl >> 8);
    TxData[6] = (uint8_t)(speedControl >> 16);
    TxData[7] = (uint8_t)(speedControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Multi_Loop_Angle_Control1
 * Description: 电机多圈位置闭环控制数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        angleControl 目标角度，对应实际位置为 0.01degree/LSB，即 36000 代表 360°
 * Return: 无
*************************************************/
void Multi_Loop_Angle_Control1(CAN_Object_t* obj, uint32_t std_id, int32_t angleControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA3;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = (uint8_t)angleControl;
    TxData[5] = (uint8_t)(angleControl >> 8);
    TxData[6] = (uint8_t)(angleControl >> 16);
    TxData[7] = (uint8_t)(angleControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Multi_Loop_Angle_Control2
 * Description: 电机多圈位置闭环控制数据发送
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        maxSpeed 电机转动的最大速度,对应实际转速 1dps/LSB，即 360 代表 360dps
 *        angleControl 目标角度，对应实际位置为 0.01degree/LSB，即 36000 代表 360°
 * Return: 无
*************************************************/
void Multi_Loop_Angle_Control2(CAN_Object_t* obj, uint32_t std_id, uint16_t maxSpeed,int32_t angleControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA4;
    TxData[1] = 0x00;
    TxData[2] = (uint8_t)maxSpeed;
    TxData[3] = (uint8_t)(maxSpeed >> 8);
    TxData[4] = (uint8_t)angleControl;
    TxData[5] = (uint8_t)(angleControl >> 8);
    TxData[6] = (uint8_t)(angleControl >> 16);
    TxData[7] = (uint8_t)(angleControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Single_Loop_Angle_Control1
 * Description: 电机单圈位置闭环控制数据发送 1
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        spinDirection 电机转动的方向,0x00 代表顺时针，0x01 代表逆时针
 *        angleControl 目标角度，对应实际位置为 0.01degree/LSB，即 36000 代表 360°
 * Return: 无
*************************************************/
void Single_Loop_Angle_Control1(CAN_Object_t* obj, uint32_t std_id, uint8_t spinDirection,uint32_t angleControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA5;
    TxData[1] = spinDirection;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = (uint8_t)angleControl;
    TxData[5] = (uint8_t)(angleControl >> 8);
    TxData[6] = (uint8_t)(angleControl >> 16);
    TxData[7] = (uint8_t)(angleControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Single_Loop_Angle_Control2
 * Description: 电机单圈位置闭环控制数据发送 2
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        spinDirection 电机转动的方向,0x00 代表顺时针，0x01 代表逆时针
 *        maxSpeed 电机转动的最大速度,对应实际转速 1dps/LSB，即 360 代表 360dps
 *        angleControl 目标角度，对应实际位置为 0.01degree/LSB，即 36000 代表 360°
 * Return: 无
*************************************************/
void Single_Loop_Angle_Control2(CAN_Object_t* obj, uint32_t std_id, uint8_t spinDirection, uint16_t maxSpeed,uint32_t angleControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA6;
    TxData[1] = spinDirection;
    TxData[2] = (uint8_t)maxSpeed;
    TxData[3] = (uint8_t)(maxSpeed >> 8);
    TxData[4] = (uint8_t)angleControl;
    TxData[5] = (uint8_t)(angleControl >> 8);
    TxData[6] = (uint8_t)(angleControl >> 16);
    TxData[7] = (uint8_t)(angleControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Increment_Angle_Control1
 * Description: 电机增量位置闭环控制命令 1
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        angleControl 目标角度，对应实际位置为 0.01degree/LSB，即 36000 代表 360°
 * Return: 无
*************************************************/
void Increment_Angle_Control1(CAN_Object_t* obj, uint32_t std_id,int32_t angleControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA7;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = (uint8_t)angleControl;
    TxData[5] = (uint8_t)(angleControl >> 8);
    TxData[6] = (uint8_t)(angleControl >> 16);
    TxData[7] = (uint8_t)(angleControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Increment_Angle_Control2
 * Description: 电机增量位置闭环控制命令 2
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        maxSpeed 电机转动的最大速度,对应实际转速 1dps/LSB，即 360 代表 360dps
 *        angleControl 目标角度，对应实际位置为 0.01degree/LSB，即 36000 代表 360°
 * Return: 无
*************************************************/
void Increment_Angle_Control2(CAN_Object_t* obj, uint32_t std_id, uint16_t maxSpeed,int32_t angleControl)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0xA8;
    TxData[1] = 0x00;
    TxData[2] = (uint8_t)maxSpeed;
    TxData[3] = (uint8_t)(maxSpeed >> 8);
    TxData[4] = (uint8_t)angleControl;
    TxData[5] = (uint8_t)(angleControl >> 8);
    TxData[6] = (uint8_t)(angleControl >> 16);
    TxData[7] = (uint8_t)(angleControl >> 24);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: PID_Read
 * Description: 读取电机PID参数命令
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 * Return: 无
*************************************************/
void PID_Read(CAN_Object_t* obj, uint32_t std_id)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x30;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: PID_RAMset
 * Description: 写入 PID 参数到 RAM 中，断电后写入参数失效
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        anglePidKp 位置环 P 参数
 *        anglePidKi 位置环 I 参数
 *        speedPidKp 速度环 P 参数
 *        speedPidKi 速度环 I 参数
 *        iqPidKp 	 转矩环 P 参数
 *        iqPidKi 	 转矩环 I 参数
 * Return: 无
*************************************************/
void PID_RAMset(CAN_Object_t* obj, uint32_t std_id, fp32 anglePidKp, fp32 anglePidKi, fp32 speedPidKp, fp32 speedPidKi, fp32 iqPidKp, fp32 iqPidKi)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x31;
    TxData[1] = 0x00;
    TxData[2] = anglePidKp;
    TxData[3] = anglePidKi;
    TxData[4] = speedPidKp;
    TxData[5] = speedPidKi;
    TxData[6] = iqPidKp;
    TxData[7] = iqPidKi;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: PID_ROMset
 * Description: 写入 PID 参数到 ROM 中，断电仍然有效
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        anglePidKp 位置环 P 参数
 *        anglePidKi 位置环 I 参数
 *        speedPidKp 速度环 P 参数
 *        speedPidKi 速度环 I 参数
 *        iqPidKp 	 转矩环 P 参数
 *        iqPidKi 	 转矩环 I 参数
 * Return: 无
*************************************************/
void PID_ROMset(CAN_Object_t* obj, uint32_t std_id, fp32 anglePidKp, fp32 anglePidKi, fp32 speedPidKp, fp32 speedPidKi, fp32 iqPidKp, fp32 iqPidKi)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x32;
    TxData[1] = 0x00;
    TxData[2] = anglePidKp;
    TxData[3] = anglePidKi;
    TxData[4] = speedPidKp;
    TxData[5] = speedPidKi;
    TxData[6] = iqPidKp;
    TxData[7] = iqPidKi;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Encoder_Read
 * Description: 读取电机编码器参数命令
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 * Return: 无
*************************************************/
void Encoder_Read(CAN_Object_t* obj, uint32_t std_id)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x90;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Encoder_Offset
 * Description: 设置电机编码器的零偏
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 *        encoderOffset 编码器零偏值，15bit 编码器的数值范围 0~32767
 * Return: 无
*************************************************/
void Encoder_Offset(CAN_Object_t* obj, uint32_t std_id,  uint16_t encoderOffset)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x91;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = (uint8_t)encoderOffset;
    TxData[7] = (uint8_t)(encoderOffset >> 8);
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Angle_Read
 * Description: 读取电机角度参数命令
 * Input: obj CAN对象指针
 *        std_id CAN发送标识符
 * Return: 无
*************************************************/
void Angle_Read(CAN_Object_t* obj, uint32_t std_id)
{
    uint8_t TxData[8] = {0};
    TxData[0] = 0x94;
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;
    TxData[4] = 0x00;
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    BSP_CAN_TransmitData(obj, std_id, TxData, 8);
}

/*************************************************
 * Function: Motor_9025_RelativePosition
 * Description: 电机编码器绝对位置计算
 * Input: ecd 编码器值
 *        offset 补偿
 * Return: 无
*************************************************/
int16_t Motor_9025_RelativePosition(int16_t ecd, int16_t offset)
{
    int16_t tmp = 0;
    if (offset >= MOTOR_9025_ENCODER_RANGE_HALF)
    {
        if (ecd > offset - MOTOR_9025_ENCODER_RANGE_HALF)
            tmp = ecd - offset;
        else
            tmp = ecd + MOTOR_9025_ENCODER_RANGE - offset;
    }
    else
    {
        if (ecd > offset + MOTOR_9025_ENCODER_RANGE_HALF)
            tmp = ecd - MOTOR_9025_ENCODER_RANGE - offset;
        else
            tmp = ecd - offset;
    }
    return tmp;
}

MotorInfo_9025_t* PlatformMotor_Pointer(void)
{
    return &platform_motor;
}


