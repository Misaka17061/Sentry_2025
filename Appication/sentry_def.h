#ifndef SENTRY_DEF_H
#define SENTRY_DEF_H
/* 包含头文件 ----------------------------------------------------------------*/
#include "Motor/motor.h"
#include "Motor/9025motor.h"

/* 宏定义 --------------------------------------------------------------------*/
/******************************************************************************
 *                              CAN通讯ID配置                                                                    *
 ******************************************************************************/
/*---------------↓ 底盘电机ID ↓---------------*/
#define CHASSIS_MOTOR_CONTROL_STD_ID    MOTOR_1TO4_CONTROL_STD_ID
#define CHASSIS_MOTOR_LF_MESSAGE_ID     MOTOR_1_FEEDBACK_ID     					//底盘左前电机   //LF
#define CHASSIS_MOTOR_RF_MESSAGE_ID     MOTOR_2_FEEDBACK_ID     					//底盘右前电机
#define CHASSIS_MOTOR_LB_MESSAGE_ID     MOTOR_3_FEEDBACK_ID    						//底盘左后电机
#define CHASSIS_MOTOR_RB_MESSAGE_ID     MOTOR_4_FEEDBACK_ID     					//底盘右后电机   //RB

#define CHASSIS_STEER_MOTOR_CONTROL_STD_ID    MOTOR_5TO8_CONTROL_STD_ID
#define CHASSIS_STEER_MOTOR_LF_MESSAGE_ID     MOTOR_5_FEEDBACK_ID     		//底盘左前云台电机
#define CHASSIS_STEER_MOTOR_RF_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     		//底盘右前云台电机
#define CHASSIS_STEER_MOTOR_LB_MESSAGE_ID     MOTOR_7_FEEDBACK_ID     		//底盘左后云台电机
#define CHASSIS_STEER_MOTOR_RB_MESSAGE_ID     MOTOR_8_FEEDBACK_ID     		//底盘右后云台电机
/*---------------↓ 云台电机ID ↓---------------*/
#define GIMBAL_MOTOR_CONTROL_STD_ID     MOTOR_5TO8_CONTROL_STD_ID 
#define GIMBAL_MOTOR_YAW_MESSAGE_ID     MOTOR_6_FEEDBACK_ID     					//Yaw云台电机
#define GIMBAL_MOTOR_PITCH_MESSAGE_ID   MOTOR_5_FEEDBACK_ID     					//Pitch云台电机
/*---------------↓ 射击电机ID ↓---------------*/
#define SHOOT_MOTOR_CONTROL_STD_ID      MOTOR_1TO4_CONTROL_STD_ID
#define FRICTION_WHEEL_1_MESSAGE_ID     MOTOR_1_FEEDBACK_ID
#define FRICTION_WHEEL_2_MESSAGE_ID     MOTOR_2_FEEDBACK_ID
#define TRIGGER_MOTOR_MESSAGE_ID        MOTOR_3_FEEDBACK_ID
#define MAGAZINE_MOTOR_MESSAGE_ID       MOTOR_4_FEEDBACK_ID
/*-------------↓ 雷达平台电机ID ↓-------------*/
#define PLATFORM_MOTOR_YAW_MESSAGE_ID   GIMBAL_MOTOR1_FEEDBACK_ID 
#define PLATFORM_MOTOR_CONTROL_STD_ID		GM_APPEND_CONTROL_STD_ID

/******************************************************************************
 *                                                     云台主控与底盘主控交互                            
 ******************************************************************************/
#define GIMBAL_DATA_STD_ID              (0x600)
#define CHASSIS_DATA_STD_ID             (0x500)
#define PLATFORM_DATA_STD_ID						(0x400)
#define REFEREE_DATA_STD_ID             (0x700)
 
/******************************************************************************
 *                                                              机械安装参数                                                                      *
 ******************************************************************************/
#define FRIC_WHEEL_RADIUS           (188.5f)    		//摩擦轮周长(mm)
#define WHEEL_RADIUS                (76)    				//轮子半径(mm)
#define WHEEL_PERIMETER             (478)   				//轮子周长(mm)
#define WHEELTRACK                  (401)   				//轮距(mm)
#define WHEELBASE                   (350)   				//轴距(mm)
#define GIMBAL_X_OFFSET             (0)     				//云台相对底盘中心X轴偏移
#define GIMBAL_Y_OFFSET             (0)    				 	//云台相对底盘中心Y轴偏移
#define PITCH_REDUCTION_RATIO       (1.0f)  				//pitch减速比
#define YAW_REDUCTION_RATIO         (1.0f)  				//yaw减速比
#define PITCH_MOTO_POSITIVE_DIR     (1.0f)  				//pitch电机安装方向
#define YAW_MOTO_POSITIVE_DIR       (1.0f)  				//yaw电机安装方向

#define STEER_MOTO_POSITIVE_DIR     (1.0f)  
#define RADIUS                      (248.195f)   		//中心半径(mm)
#define STEER_REDUCTION_RATIO       (1.0f)  

 
/******************************************************************************
 *                                                                   移动控制                                                                         *
 ******************************************************************************/
#define MAX_CHASSIS_VX_SPEED        (4900.0f)
#define MAX_CHASSIS_VY_SPEED        (4900.0f)
#define MAX_CHASSIS_VW_SPEED        (500.0f)
/*-----------------↓ 遥控 ↓-----------------*/
#define RC_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度(mm/s)
#define RC_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     //旋转最大速度(deg/s)
#define RC_GIMBAL_MOVE_RATIO_PIT    -0.00025f       				 //pitch移动比例
#define RC_GIMBAL_MOVE_RATIO_YAW    0.00025f         				 //yaw移动比例   //0.001
#define RC_PLATFORM_MOVE_RATIO_YAW  0.00075f         				 //yaw移动比例   //0.001
/*---------------↓ 鼠标键盘 ↓---------------*/
#define KB_CHASSIS_MAX_SPEED_X      MAX_CHASSIS_VX_SPEED     //X轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_Y      MAX_CHASSIS_VX_SPEED     //Y轴方向最大速度
#define KB_CHASSIS_MAX_SPEED_R      MAX_CHASSIS_VW_SPEED     //旋转最大速度
#define KB_GIMBAL_MOVE_RATIO_PIT    0.010f       						 //pitch移动比例
#define KB_GIMBAL_MOVE_RATIO_YAW    0.015f        					 //yaw移动比例

#define VS_GIMBAL_MOVE_RATIO_YAW    0.0375f         				 //yaw移动比例
#define VS_GIMBAL_MOVE_RATIO_PIT    0.0375f        					 //pitch移动比例

#define CHASSIS_ACCEL_TIME      1500  //ms            			 底盘加速度时间
#define ROTATE_ACCEL_TIME       3000  //ms            			 旋转加速度时间
#define CHASSIS_SHIFT_ACCEL_TIME      2161  //ms             底盘加速度时间


/******************************************************************************
 *                                                                   任务周期                                                                         *
 ******************************************************************************/
/*---------------↓ 通用任务 ↓---------------*/
#define START_TASK_PERIOD           100
#define IMU_TASK_PERIOD             5
#define CONSOLE_TASK_PERIOD         5
#define COMM_TASK_PERIOD            1
#define DETECT_TASK_PERIOD          20
/*---------------↓ 底盘任务 ↓---------------*/
#define CHASSIS_TASK_PERIOD         5
#define DECISION_TASK_PERIOD        5
/*---------------↓ 云台任务 ↓---------------*/
#define GIMBAL_TASK_PERIOD          2
#define SHOOT_TASK_PERIOD           4
#define PLATFORM_TASK_PERIOD        4
#endif 
