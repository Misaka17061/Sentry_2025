/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "9025motor.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/
MotorInfo_9025_t platform_motor;
/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
/*************************************************
 * Function: Motor_9025_DataParse
 * Description: 9025����ص����ݴ���
 * Input: ptr �����Ϣָ��
 *        data ����ָ��
 * Return: ��
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
 * Description: ����ر����ݷ���
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 * Return: ��
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
 * Description: ����������ݷ���
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 * Return: ��
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
 * Description: ��������������ݷ���
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        powerControl �������ֵ����ֵ��Χ-850~850
 * Return: ��
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
 * Description: ����ٶȿ������ݷ���
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        speedControl ����ٶ�ֵ����Ӧʵ��ת��Ϊ0.01dps/LSB
 * Return: ��
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
 * Description: �����Ȧλ�ñջ��������ݷ���
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        angleControl Ŀ��Ƕȣ���Ӧʵ��λ��Ϊ 0.01degree/LSB���� 36000 ���� 360��
 * Return: ��
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
 * Description: �����Ȧλ�ñջ��������ݷ���
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        maxSpeed ���ת��������ٶ�,��Ӧʵ��ת�� 1dps/LSB���� 360 ���� 360dps
 *        angleControl Ŀ��Ƕȣ���Ӧʵ��λ��Ϊ 0.01degree/LSB���� 36000 ���� 360��
 * Return: ��
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
 * Description: �����Ȧλ�ñջ��������ݷ��� 1
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        spinDirection ���ת���ķ���,0x00 ����˳ʱ�룬0x01 ������ʱ��
 *        angleControl Ŀ��Ƕȣ���Ӧʵ��λ��Ϊ 0.01degree/LSB���� 36000 ���� 360��
 * Return: ��
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
 * Description: �����Ȧλ�ñջ��������ݷ��� 2
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        spinDirection ���ת���ķ���,0x00 ����˳ʱ�룬0x01 ������ʱ��
 *        maxSpeed ���ת��������ٶ�,��Ӧʵ��ת�� 1dps/LSB���� 360 ���� 360dps
 *        angleControl Ŀ��Ƕȣ���Ӧʵ��λ��Ϊ 0.01degree/LSB���� 36000 ���� 360��
 * Return: ��
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
 * Description: �������λ�ñջ��������� 1
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        angleControl Ŀ��Ƕȣ���Ӧʵ��λ��Ϊ 0.01degree/LSB���� 36000 ���� 360��
 * Return: ��
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
 * Description: �������λ�ñջ��������� 2
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        maxSpeed ���ת��������ٶ�,��Ӧʵ��ת�� 1dps/LSB���� 360 ���� 360dps
 *        angleControl Ŀ��Ƕȣ���Ӧʵ��λ��Ϊ 0.01degree/LSB���� 36000 ���� 360��
 * Return: ��
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
 * Description: ��ȡ���PID��������
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 * Return: ��
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
 * Description: д�� PID ������ RAM �У��ϵ��д�����ʧЧ
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        anglePidKp λ�û� P ����
 *        anglePidKi λ�û� I ����
 *        speedPidKp �ٶȻ� P ����
 *        speedPidKi �ٶȻ� I ����
 *        iqPidKp 	 ת�ػ� P ����
 *        iqPidKi 	 ת�ػ� I ����
 * Return: ��
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
 * Description: д�� PID ������ ROM �У��ϵ���Ȼ��Ч
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        anglePidKp λ�û� P ����
 *        anglePidKi λ�û� I ����
 *        speedPidKp �ٶȻ� P ����
 *        speedPidKi �ٶȻ� I ����
 *        iqPidKp 	 ת�ػ� P ����
 *        iqPidKi 	 ת�ػ� I ����
 * Return: ��
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
 * Description: ��ȡ�����������������
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 * Return: ��
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
 * Description: ���õ������������ƫ
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 *        encoderOffset ��������ƫֵ��15bit ����������ֵ��Χ 0~32767
 * Return: ��
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
 * Description: ��ȡ����ǶȲ�������
 * Input: obj CAN����ָ��
 *        std_id CAN���ͱ�ʶ��
 * Return: ��
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
 * Description: �������������λ�ü���
 * Input: ecd ������ֵ
 *        offset ����
 * Return: ��
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


