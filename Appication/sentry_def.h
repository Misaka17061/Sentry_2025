#ifndef SENTRY_DEF_H
#define SENTRY_DEF_H
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "Motor/motor.h"
#include "Motor/9025motor.h"

/* �궨�� --------------------------------------------------------------------*/
/******************************************************************************
 *                              CANͨѶID����                                                                    *
 ******************************************************************************/
/*---------------�� ���̵��ID ��---------------*/
#define CHASSIS_MOTOR_CONTROL_STD_ID    MOTOR_1TO4_CONTROL_STD_ID
#define CHASSIS_MOTOR_LF_MESSAGE_ID     MOTOR_1_FEEDBACK_ID     					//������ǰ���   //LF
#define CHASSIS_MOTOR_RF_MESSAGE_ID     MOTOR_2_FEEDBACK_ID     					//������ǰ���
#define CHASSIS_MOTOR_LB_MESSAGE_ID     MOTOR_3_FEEDBACK_ID    						//���������
#define CHASSIS_MOTOR_RB_MESSAGE_ID     MOTOR_4_FEEDBACK_ID     					//�����Һ���   //RB

#define CHASSIS_STEER_MOTOR_CONTROL_STD_ID    MOTOR_5TO8_CONTROL_STD_ID
#define CHASSIS_STEER_MOTOR_LF_MESSAGE_ID     MOTOR_5_FEEDBACK_ID     		//������ǰ��̨���
#define CHASSIS_STEER_MOTOR_RF_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     		//������ǰ��̨���
#define CHASSIS_STEER_MOTOR_LB_MESSAGE_ID     MOTOR_7_FEEDBACK_ID     		//���������̨���
#define CHASSIS_STEER_MOTOR_RB_MESSAGE_ID     MOTOR_8_FEEDBACK_ID     		//�����Һ���̨���
/*---------------�� ��̨���ID ��---------------*/
#define GIMBAL_MOTOR_CONTROL_STD_ID     MOTOR_5TO8_CONTROL_STD_ID 
#define GIMBAL_MOTOR_YAW_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     					//Yaw��̨���
#define GIMBAL_MOTOR_PITCH_MESSAGE_ID   MOTOR_5_FEEDBACK_ID     					//Pitch��̨���
/*---------------�� ������ID ��---------------*/
#define SHOOT_MOTOR_CONTROL_STD_ID      MOTOR_1TO4_CONTROL_STD_ID
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID
/*-------------�� �״�ƽ̨���ID ��-------------*/
#define PLATFORM_MOTOR_YAW_MESSAGE_ID   GIMBAL_MOTOR1_FEEDBACK_ID 
#define PLATFORM_MOTOR_CONTROL_STD_ID		GM_APPEND_CONTROL_STD_ID

/******************************************************************************
 *                                                     ��̨������������ؽ���                            
 ******************************************************************************/
#define GIMBAL_DATA_STD_ID              (0x600)
#define CHASSIS_DATA_STD_ID             (0x500)
#define PLATFORM_DATA_STD_ID						(0x400)
#define REFEREE_DATA_STD_ID             (0x700)
 
/******************************************************************************
 *                                                              ��е��װ����                                                                      *
 ******************************************************************************/
#define FRIC_WHEEL_RADIUS           (188.5f)    		//Ħ�����ܳ�(mm)
#define WHEEL_RADIUS                (76)    				//���Ӱ뾶(mm)
#define WHEEL_PERIMETER             (478)   				//�����ܳ�(mm)
#define WHEELTRACK                  (401)   				//�־�(mm)
#define WHEELBASE                   (350)   				//���(mm)
#define GIMBAL_X_OFFSET             (0)     				//��̨��Ե�������X��ƫ��
#define GIMBAL_Y_OFFSET             (0)    				 	//��̨��Ե�������Y��ƫ��
#define PITCH_REDUCTION_RATIO       (1.0f)  				//pitch���ٱ�
#define YAW_REDUCTION_RATIO         (1.0f)  				//yaw���ٱ�
#define PITCH_MOTO_POSITIVE_DIR     (1.0f)  				//pitch�����װ����
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  				//yaw�����װ����

#define STEER_MOTO_POSITIVE_DIR     (1.0f)  
#define RADIUS                      (248.195f)   		//���İ뾶(mm)
#define STEER_REDUCTION_RATIO       (1.0f)  

 
/******************************************************************************
 *                                                                   �ƶ�����                                                                         *
 ******************************************************************************/
#define MAX_CHASSIS_VX_SPEED        (4900.0f)
#define MAX_CHASSIS_VY_SPEED        (4900.0f)
#define MAX_CHASSIS_VW_SPEED        (500.0f)
/*-----------------�� ң�� ��-----------------*/
#define RC_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X�᷽������ٶ�(mm/s)
#define RC_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y�᷽������ٶ�(mm/s)
#define RC_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     //��ת����ٶ�(deg/s)
#define RC_GIMBAL_MOVE_RATIO_PIT    -0.00025f       				 //pitch�ƶ�����
#define RC_GIMBAL_MOVE_RATIO_YAW    0.00025f         				 //yaw�ƶ�����   //0.001
#define RC_PLATFORM_MOVE_RATIO_YAW  0.00075f         				 //yaw�ƶ�����   //0.001
/*---------------�� ������ ��---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X�᷽������ٶ�
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y�᷽������ٶ�
#define KB_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     //��ת����ٶ�
#define KB_GIMBAL_MOVE_RATIO_PIT    0.010f       						 //pitch�ƶ�����
#define KB_GIMBAL_MOVE_RATIO_YAW    0.015f        					 //yaw�ƶ�����

#define VS_GIMBAL_MOVE_RATIO_YAW    0.0375f         				 //yaw�ƶ�����
#define VS_GIMBAL_MOVE_RATIO_PIT    0.0375f        					 //pitch�ƶ�����

#define CHASSIS_ACCEL_TIME      1500  //ms            			 ���̼��ٶ�ʱ��
#define ROTATE_ACCEL_TIME       3000  //ms            			 ��ת���ٶ�ʱ��
#define CHASSIS_SHIFT_ACCEL_TIME      2161  //ms             ���̼��ٶ�ʱ��


/******************************************************************************
 *                                                                   ��������                                                                         *
 ******************************************************************************/
/*---------------�� ͨ������ ��---------------*/
#define START_TASK_PERIOD           100
#define IMU_TASK_PERIOD             5
#define CONSOLE_TASK_PERIOD         5
#define COMM_TASK_PERIOD            1
#define DETECT_TASK_PERIOD          20
/*---------------�� �������� ��---------------*/
#define CHASSIS_TASK_PERIOD         5
#define DECISION_TASK_PERIOD        5
/*---------------�� ��̨���� ��---------------*/
#define GIMBAL_TASK_PERIOD          2
#define SHOOT_TASK_PERIOD           4
#define PLATFORM_TASK_PERIOD        4
#endif 
