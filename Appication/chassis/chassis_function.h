#ifndef	CHASSIS_FUNTION_H
#define	CHASSIS_FUNTION_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "chassis_app.h"
#include "arm_math.h"
#include "user_lib.h"
#include "pid.h"
#include "ramp.h"

/* ���Ͷ��� ------------------------------------------------------------------*/

/* �궨�� --------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void Chassis_MoveTransform(ChassisHandle_t* chassis_handle, fp32* chassis_vx, fp32* chassis_vy);
void Mecanum_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw);
void Chassis_LimitPower(ChassisHandle_t* chassis_handle);
void Chassis_ControlCalc(ChassisHandle_t* chassis_handle);

#endif
