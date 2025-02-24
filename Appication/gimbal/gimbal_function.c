/* 包含头文件 ----------------------------------------------------------------*/
#include "gimbal_function.h"
#include "pid.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/

/* 扩展变量 ------------------------------------------------------------------*/
extern VisionDatabase_t vision_data;
extern Console_t console;
/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void GimbalMotorChangeProtect(M6020_Motor_t* motor)
{
    if (motor->last_mode != motor->mode)
    {
        if(motor->mode == RAW_VALUE_MODE)
        {
            motor->given_value = motor->current_set;
        }
        else if (motor->mode == GYRO_MODE)
        {
            motor->given_value = motor->sensor.gyro_angle;
        }
        else if (motor->mode == ENCONDE_MODE)
        {
            motor->given_value = motor->sensor.relative_angle;
        }
    }
    motor->last_mode = motor->mode;
}

void GimbalMotorControl(M6020_Motor_t* motor)
{
    GimbalMotorChangeProtect(motor);
    if (motor->mode == RAW_VALUE_MODE)
    {
        motor->current_set = motor->given_value;
        dpid_clear(&motor->dpid);
    }
    else if(motor->mode == GYRO_MODE && (console.gimbal_cmd == GIMBAL_VISION_AIM_CMD || console.gimbal_cmd == GIMBAL_SENTRY_CMD))
    {
        motor->current_set = Vision_ESO_DoublePID_Calc(&motor->dpid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance,
											 &ESO_6020);
    }
		else if(motor->mode == GYRO_MODE)
    {
        motor->current_set = DoublePID_Calc(&motor->dpid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance);
    }
    else if(motor->mode == ENCONDE_MODE)
    {
        motor->current_set = DoublePID_Calc(&motor->dpid,
                                             motor->given_value,
                                             motor->sensor.relative_angle,
                                             motor->sensor.palstance);
    }
}

void PlatformMotorControl(Motor_9025_t* motor)
{
	    if (motor->last_mode != motor->mode)
    {
        if(motor->mode == RAW_VALUE_MODE)
        {
            motor->given_value = motor->powerControl;
        }
        else if (motor->mode == GYRO_MODE)
        {
            motor->given_value = motor->sensor.gyro_angle;
        }
        else if (motor->mode == ENCONDE_MODE)
        {
            motor->given_value = motor->sensor.relative_angle;
        }
    }
    motor->last_mode = motor->mode;
		if(motor->mode == GYRO_MODE && console.gimbal_cmd == COMPUTER_VISION_CONTROL_CMD)
		{
				motor->powerControl = ESO_DoublePID_Calc(&motor->dpid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance,
											 &ESO_9025);
		}
		else if(motor->mode == GYRO_MODE)
		{
				motor->powerControl = DoublePID_Calc(&motor->dpid,
                                             motor->given_value,
                                             motor->sensor.gyro_angle,
                                             motor->sensor.palstance);
		}
		else if(motor->mode == ENCONDE_MODE)
		{
				motor->powerControl = DoublePID_Calc(&motor->dpid,
                                             motor->given_value,
                                             motor->sensor.relative_angle,
                                             motor->sensor.palstance);
		}
}

float Vision_ESO_DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb, ESO_t *ESO)
{
    dpid->outer_ref = outer_ref;
    dpid->outer_fdb = outer_fdb;
    pid_calc(&dpid->outer_pid, dpid->outer_fdb, dpid->outer_ref);
		dpid->inter_fdb = inter_fdb;
		if(vision_data.move_state == 2)
		{
			dpid->inter_ref = dpid->outer_pid.out * 2 - ESO->z2;
		}
    else
    {
			dpid->inter_ref = dpid->outer_pid.out - ESO->z2 + vision_data.palstance;
		}
    pid_calc(&dpid->inter_pid, dpid->inter_fdb, dpid->inter_ref);
		dpid->inter_pid.out -= ESO->z3;
    return dpid->inter_pid.out;
}

float ESO_DoublePID_Calc(Double_PID_t* dpid, float outer_ref, float outer_fdb, float inter_fdb, ESO_t *ESO)
{
    dpid->outer_ref = outer_ref - ESO->z1;
    dpid->outer_fdb = outer_fdb;
    pid_calc(&dpid->outer_pid, dpid->outer_fdb, dpid->outer_ref);
		dpid->inter_fdb = inter_fdb;
		dpid->inter_ref = dpid->outer_pid.out - ESO->z2;
    pid_calc(&dpid->inter_pid, dpid->inter_fdb, dpid->inter_ref);
		dpid->inter_pid.out -= ESO->z3;
    return dpid->inter_pid.out;
}

fp32 AngleTransform(fp32 target_angle, fp32 gyro_angle)
{
		
    float offset = 0,now = 0, target = 0;

    ANGLE_LIMIT_180(target, target_angle);
    ANGLE_LIMIT_180(now, gyro_angle);

    offset = target - now;
    if (offset > 180)
    {
        offset = offset - 360;
    }
    else if (offset < -180)
    {
        offset = offset + 360;
    }
    return gyro_angle + offset;
}




