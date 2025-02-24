/* 包含头文件 ----------------------------------------------------------------*/
#include "chassis_function.h"
#include "sentry_def.h"
#include "referee_system.h"
#include "detect_task.h"
#include "pid.h"

/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/
#define MAX_WHEEL_RPM   								M3508_MAX_RPM

#define POWER_LIMIT        							80.0f
#define WARNING_POWER       						40.0f
#define WARNING_POWER_BUFF  						60.0f
#define MAX_TOTAL_CURRENT_LIMIT         64000.0f    //16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f

#define M3508S_REDUCTION_RATIO       		(1.0f/13.8f)        //3508减速比

#define ENCODER_MAX 8192

/* 私有变量 ------------------------------------------------------------------*/
static fp32 total_current_limit = 0.0f;
static fp32 chassis_power = 0.0f;
static fp32 chassis_power_buffer = 0.0f;

float Mini_Arc[4]={0},Max_Arc[4]={0};

static fp32 target_angle[4]={0};

static float now_angle[4]={0,0,0,0};
static float new_target_angle[4]={0};
static float back_target_angle[4];
static float angle_error[4];
static float solute_angle_error[4];
static float	first_Mini_Arc[4]={0}; 

/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/
static float encoder_to_radians(int current_ecd, int offset_ecd);

/* 函数体 --------------------------------------------------------------------*/
void Chassis_MoveTransform(ChassisHandle_t* chassis_handle, fp32* chassis_vx, fp32* chassis_vy)  //底盘云台跟随解算
{
    static fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
		
		if(chassis_handle->ctrl_mode == CHASSIS_SPIN)		//左加右减
		{
			sin_yaw = arm_sin_f32((chassis_handle->platform_yaw_ecd_angle + 40) / RADIAN_COEF);
			cos_yaw = arm_cos_f32((chassis_handle->platform_yaw_ecd_angle + 40) / RADIAN_COEF);
		}
		if(chassis_handle->ctrl_mode == CHASSIS_SENTRY)		//左加右减
		{
			sin_yaw = arm_sin_f32((chassis_handle->platform_yaw_ecd_angle + 13) / RADIAN_COEF);
			cos_yaw = arm_cos_f32((chassis_handle->platform_yaw_ecd_angle + 13) / RADIAN_COEF);
		}
    else
		{
			sin_yaw = arm_sin_f32((chassis_handle->platform_yaw_ecd_angle) / RADIAN_COEF);
			cos_yaw = arm_cos_f32((chassis_handle->platform_yaw_ecd_angle) / RADIAN_COEF);
		}

    *chassis_vx = cos_yaw * chassis_handle->vx + sin_yaw * chassis_handle->vy;
    *chassis_vy =-sin_yaw * chassis_handle->vx + cos_yaw * chassis_handle->vy;
}

int16_t fbs16(int16_t xx)
{
		if(xx<0) return -xx;
		else return xx;
}

void Chassis_LimitPower(ChassisHandle_t* chassis_handle)
{
		fp32 total_current = 0.0f;
    uint8_t robot_id = RefereeSystem_GetRobotID();

    if (robot_id == 0 || CheckDeviceIsOffline(OFFLINE_REFEREE_SYSTEM))
    {
        total_current_limit = 8000;
    }
    else if (robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER)
    {
        total_current_limit = MAX_TOTAL_CURRENT_LIMIT;
    }
    else
    {
        chassis_power = RefereeSystem_PowerHeatData_Pointer()->chassis_power;
        chassis_power_buffer = RefereeSystem_PowerHeatData_Pointer()->chassis_power_buffer;
               
        if(chassis_power_buffer < WARNING_POWER_BUFF)
				{
						fp32 power_scale;
						if(chassis_power_buffer > 5.0f)
						{
								power_scale = chassis_power_buffer / WARNING_POWER_BUFF;
						}
						else
						{
               
								power_scale = 5.0f / WARNING_POWER_BUFF;
						}
           
								total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
				}
				else
				{
						total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
				}

    }

    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fbs16(chassis_handle->chassis_motor[i].current_set);
    }

    if(total_current > total_current_limit)
    {
			  fp32 current_scale = total_current_limit / total_current;            //系数<1减速，系数>1加速
        chassis_handle->chassis_motor[0].current_set *= current_scale;
        chassis_handle->chassis_motor[1].current_set *= current_scale;
        chassis_handle->chassis_motor[2].current_set *= current_scale;
        chassis_handle->chassis_motor[3].current_set *= current_scale;
    }
	
		chassis_power = fbs16(chassis_handle->chassis_motor[0].motor_info->given_current/8)
                      + fbs16(chassis_handle->chassis_motor[1].motor_info->given_current/8)
                      + fbs16(chassis_handle->chassis_motor[2].motor_info->given_current/8)
											+ fbs16(chassis_handle->chassis_motor[3].motor_info->given_current/8)
											+ fbs16(chassis_handle->chassis_steer_motor[0].motor_info->given_current/8)
                      + fbs16(chassis_handle->chassis_steer_motor[1].motor_info->given_current/8)
                      + fbs16(chassis_handle->chassis_steer_motor[2].motor_info->given_current/8)
                      + fbs16(chassis_handle->chassis_steer_motor[3].motor_info->given_current/8);
		if(chassis_power > 3072)                          //舵轮防通道0过载
		{
        chassis_handle->chassis_motor[0].current_set /= 8.0f;
				chassis_handle->chassis_motor[1].current_set /= 8.0f;
        chassis_handle->chassis_motor[2].current_set /= 8.0f;
        chassis_handle->chassis_motor[3].current_set /= 8.0f; 
        chassis_handle->chassis_steer_motor[0].current_set /= 2.0f;
        chassis_handle->chassis_steer_motor[1].current_set /= 2.0f;
        chassis_handle->chassis_steer_motor[2].current_set /= 2.0f;
        chassis_handle->chassis_steer_motor[3].current_set /= 2.0f; 		
		}  
}

void Steer_Speed_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw) //舵轮轮速解算
{
    fp32 theta = atan(1.0/1.0);                     //程序换算角度为弧度不能直接使用！！
    fp32 steer_vw = chassis_vw * (3.14/180);
		fp32 wheel_rpm_ratio;
		fp32 wheel_rpm[4];
    fp32 max = 0;
	
		wheel_rpm_ratio = 60.0f/(WHEEL_PERIMETER * M3508_REDUCTION_RATIO);    //舵轮转速转换
	
		wheel_rpm[0]
		= -sqrt(pow(chassis_vx + steer_vw*RADIUS*sin(theta),2)                
			     + pow(chassis_vy + steer_vw*RADIUS*cos(theta),2))*
				   wheel_rpm_ratio;      //对应华南理工公式  Vx1-Vw1sin45*RADIUS    
			
		wheel_rpm[1]
		= sqrt(	pow(chassis_vx - steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy + steer_vw*RADIUS*cos(theta),2))* 
	          wheel_rpm_ratio;      	 //对应华南理工公式  Vx2+Vw2sin45*RADIUS  																																																																								
			
		wheel_rpm[2]
		= -sqrt(	pow(chassis_vx + steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy - steer_vw*RADIUS*cos(theta),2))* 
						wheel_rpm_ratio;        //对应华南理工公式  Vy1-Vw1cos45*RADIUS  
																																																																																
		wheel_rpm[3]
		= sqrt(	pow(chassis_vx - steer_vw*RADIUS*sin(theta),2)
						+	pow(chassis_vy - steer_vw*RADIUS*cos(theta),2))*
						wheel_rpm_ratio;        //对应华南理工公式  Vy2+Vw2cos45*RADIUS 
			
/*************************寻找舵最近角度从而降低功率************************/			
			//find max item
		for (uint8_t i = 0; i < 4; i++)
		{
				wheel_rpm[i]=wheel_rpm[i]*cos(Mini_Arc[i]);
				if(chassis_handle->turnFlag[i]==1)
				{
						wheel_rpm[i] = -wheel_rpm[i];
				}
				else
				{
						wheel_rpm[i] = wheel_rpm[i];
				}
		}
			
		for (uint8_t i = 0; i < 4; i++)
    {
        if (fabs(wheel_rpm[i]) > max)
        {
            max = fabs(wheel_rpm[i]);
        }
    }

    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_rpm[i] *= rate;
        }
    }

		memcpy(chassis_handle->wheel_rpm, wheel_rpm, 4 * sizeof(fp32));
}

void Steer_angle_change(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)  
{
		float theta = atan(1.0/1.0);
		fp32 wheel_angle[4];      
		fp32 steer_vw = chassis_vw * (3.14/180);
		if((chassis_vx==0)&&(chassis_vy==0)&&(chassis_vw==0))              
		{
				memcpy(wheel_angle, chassis_handle->lastSteeringAngletarget, 4 * sizeof(fp32));
		}
		else      //舵轮角度解算       
		{ //解算出(-PI,PI)弧度
				wheel_angle[0]=atan2((chassis_vy+steer_vw*RADIUS*sin(theta)),
							(chassis_vx+steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy1-vw1cos45)/(vx1-vw1sin45))
				wheel_angle[1]=atan2((chassis_vy+steer_vw*RADIUS*sin(theta)),
							(chassis_vx-steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy2-vw2cos45)/(vx2+vw2sin45)) 
				wheel_angle[2]=atan2((chassis_vy-steer_vw*RADIUS*sin(theta)),
							(chassis_vx+steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy3+vw3cos45)/(vx3+vw3sin45)) 
				wheel_angle[3]=atan2((chassis_vy-steer_vw*RADIUS*sin(theta)),
							(chassis_vx-steer_vw*RADIUS*cos(theta)));       //对应华南理工公式VX=tan-1((vy4+vw4cos45)/(vx4-vw4sin45)) 
		}
		for(uint8_t i=0;i<4;i++)
		{
				
		}
		memcpy(chassis_handle->lastSteeringAngletarget, wheel_angle, 4 * sizeof(fp32));
		memcpy(target_angle, wheel_angle, 4 * sizeof(fp32));  
}


void CalculateNowangle(ChassisHandle_t* chassis_handle)    
{	
		for(uint8_t i=0;i<4;i++)
		{
				chassis_handle->steeringAngle[i] = encoder_to_radians(chassis_handle->chassis_steer_motor[i].motor_info->ecd,
																															chassis_handle->chassis_steer_motor[i].offset_ecd);//将当前角限制在(-PI,PI)，绝对坐标系-绝对坐标系（以初始角作为绝对坐标系）
				chassis_handle->steeringAngle[i] = chassis_handle->steeringAngle[i]/PI*180;
				chassis_handle->chassis_steer_motor[i].sensor.relative_angle = chassis_handle->steeringAngle[i];     //////////////////////////////////
		}
}

void CalculateTargeAngle(ChassisHandle_t* chassis_handle)   
{

	
		for(uint8_t i=0;i<4;i++)
		{
//		target_angle[i]=target_angle[i]/180.0*PI;
				now_angle[i]=chassis_handle->chassis_steer_motor[i].sensor.relative_angle/180.0f*PI;
/*******************优劣弧处理*************/
				angle_error[i] =target_angle[i] - now_angle[i];//角度范围需要规范!
				solute_angle_error[i]=fabs(angle_error[i]);//为正值
				Mini_Arc[i]=solute_angle_error[i];//第一层优弧(0,360)
		    Max_Arc[i]=2*PI-Mini_Arc[i];//第一层劣弧(0,360)
				if(fabs(Mini_Arc[i])>fabs(Max_Arc[i]))//得出实际优弧
						Mini_Arc[i]=Max_Arc[i];
				first_Mini_Arc[i]=Mini_Arc[i];//第一层最终优弧(0,180)
///*优弧绝对值小于PI/2,则无需改动，大于则要继续限制*/		
				if(fabs(Mini_Arc[i])>PI/2)
				{	
						Mini_Arc[i]=PI-Mini_Arc[i];//第二层优弧值域(0,90) 
						chassis_handle->turnFlag[i]=1;
				}
				else chassis_handle->turnFlag[i]=0;//速度反向
					
/*******对目标角的对角进行限制*********/
        back_target_angle[i]=target_angle[i]+PI;
        if(back_target_angle[i]>PI)
						back_target_angle[i]-=2*PI;

/****确定弧是逆时针还是顺时针********/
		
		if(angle_error[i]>0&&fabs(angle_error[i])<=PI/2) //顺时针
        new_target_angle[i]=now_angle[i]+Mini_Arc[i];
	
		if(angle_error[i]<=0&&fabs(angle_error[i])<=PI/2) //逆时针
		 new_target_angle[i]=now_angle[i]-Mini_Arc[i];

		if(angle_error[i]>0&&fabs(angle_error[i])>PI/2)
		{
			new_target_angle[i]=now_angle[i]-Mini_Arc[i];  //逆时针
			if((new_target_angle[i]!=back_target_angle[i])&&(first_Mini_Arc[i]>PI/2))
			{
				new_target_angle[i]=back_target_angle[i];  //顺时针
			if(fbs16(new_target_angle[i]-now_angle[i]>PI*3/2))//过零处理
				new_target_angle[i]-=2*PI;
			}
		}	

		if(angle_error[i]<=0&&fabs(angle_error[i])>PI/2)
		{
			new_target_angle[i]=now_angle[i]+Mini_Arc[i];  //顺时针	
			if((new_target_angle[i]!=back_target_angle[i])&&(first_Mini_Arc[i]>PI/2))
			{
				
				new_target_angle[i]=back_target_angle[i];  //顺时针
			if(fbs16(new_target_angle[i]-now_angle[i]>PI*3/2))//过零处理
				new_target_angle[i]+=2*PI;
			}
		}			
//角度更新	
	new_target_angle[i]=new_target_angle[i]*180.0f/PI;
	}
	memcpy(chassis_handle->steeringAngleTarget, new_target_angle, 4 * sizeof(fp32));
}

float encoder_to_radians(int current_ecd, int offset_ecd) 
{
    // 计算相对编码器位置
    int relative_ecd = current_ecd - offset_ecd;
    
    // 正常化相对位置
    if(relative_ecd < 0) {
        relative_ecd += ENCODER_MAX;
    } else if(relative_ecd >= ENCODER_MAX) {
        relative_ecd -= ENCODER_MAX;
    }

    // 将相对位置映射到范围[0, ENCODER_MAX)内
    // 这里使用的是对称映射方法，以保证输出在(-PI, PI)范围内
    if(relative_ecd > ENCODER_MAX / 2) {
        relative_ecd -= ENCODER_MAX;
    }
    
    // 相对位置转为弧度值(-PI,PI)
    float radian = relative_ecd * (2 * PI / ENCODER_MAX);
    
    return radian;
}

void Steer_Calculate(ChassisHandle_t* chassis_handle, fp32 chassis_vx, fp32 chassis_vy, fp32 chassis_vw)   //舵轮底盘解算
{	
	CalculateNowangle(chassis_handle);
	Steer_angle_change(chassis_handle,chassis_vx,chassis_vy,chassis_vw);   
	CalculateTargeAngle(chassis_handle);
	Steer_Speed_Calculate(chassis_handle,chassis_vx,chassis_vy,chassis_vw); 
}

void Chassis_Steer_MotorControl(M6020_Motor_t* motor)   //底盘PID控制
{    
     motor->current_set = DoublePID_Calc(&motor->dpid,
                                          motor->given_value,
                                          motor->sensor.relative_angle,
                                          motor->sensor.palstance);
}

void Chassis_ControlCalc(ChassisHandle_t* chassis_handle)
{
    static float chassis_vx = 0.0f, chassis_vy = 0.0f;
    Chassis_MoveTransform(chassis_handle, &chassis_vx, &chassis_vy);
    Steer_Calculate(chassis_handle, chassis_vx, chassis_vy, chassis_handle->vw);
}


