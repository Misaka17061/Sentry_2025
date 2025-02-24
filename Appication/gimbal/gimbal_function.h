#ifndef GIMBAL_FUNCTION_H
#define GIMBAL_FUNCTION_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "gimbal_app.h"
#include "user_lib.h"
/* ���Ͷ��� ------------------------------------------------------------------*/

/* �궨�� --------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void GimbalMotorChangeProtect(M6020_Motor_t* motor);
void GimbalMotorControl(M6020_Motor_t* motor);
float Vision_ESO_DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb, ESO_t *ESO);
float ESO_DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb, ESO_t *ESO);
fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle);
void PlatformMotorControl(Motor_9025_t* motor);
#endif  // GIMBAL_FUNCTION_H
