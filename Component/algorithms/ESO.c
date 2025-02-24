/* 包含头文件 ----------------------------------------------------------------*/
#include "ESO.h"
/* 私有类型定义 --------------------------------------------------------------*/

/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
ESO_t ESO_6020;
ESO_t ESO_9025;
/* 扩展变量 ------------------------------------------------------------------*/

/* 私有函数原形 --------------------------------------------------------------*/

/* 函数体 --------------------------------------------------------------------*/
void ESO_6020_init(void)
{
	ESO_6020.e = 0,ESO_6020.z1 = 0,ESO_6020.z2 = 0,ESO_6020.z3 = 0,ESO_6020.last_z1 = 0,ESO_6020.last_z2 = 0,ESO_6020.last_z3 = 0,ESO_6020.u = 0;
	ESO_6020.y = 0;
	ESO_6020.last_y = 0;
	ESO_6020.h = 0.002;						//控制周期				s
	ESO_6020.w = 10;						//扰动带宽
	ESO_6020.K = 0.001;						//电压系数	
	ESO_6020.Km = 0.741;					//转矩常数				N*M/A
	ESO_6020.Ke = 13.33/60;					//转速常数				rps/V
	ESO_6020.J = 0.1;						//转动惯量				kg*m^2
	ESO_6020.R = 1.8;						//电机电阻				R
	ESO_6020.b = 2;							//系统阶数
	ESO_6020.deta1 = 3 * ESO_6020.w;
	ESO_6020.deta2 = 3 * pow(ESO_6020.w,2);
	ESO_6020.deta3 = 3 * pow(ESO_6020.w,3);
}

void ESO_9025_init(void)
{
	ESO_9025.e = 0,ESO_9025.z1 = 0,ESO_9025.z2 = 0,ESO_9025.z3 = 0,ESO_9025.last_z1 = 0,ESO_9025.last_z2 = 0,ESO_9025.last_z3 = 0,ESO_9025.u = 0;
	ESO_9025.y = 0;
	ESO_9025.last_y = 0;
	ESO_9025.h = 0.002;						//控制周期				s
	ESO_9025.w = 5;							//扰动带宽
	ESO_9025.K = 0.001;						//电压系数	
	ESO_9025.Km = 1.40;						//转矩常数				N*M/A
	ESO_9025.Ke = 0.8;						//转速常数				rps/V
	ESO_9025.J = 0.4;						//转动惯量				kg*m^2
	ESO_9025.R = 9;							//电机电阻				R
	ESO_9025.b = 2;							//系统阶数
	ESO_9025.deta1 = 3 * ESO_9025.w;
	ESO_9025.deta2 = 3 * pow(ESO_9025.w,2);
	ESO_9025.deta3 = 3 * pow(ESO_9025.w,3);
}

int32_t ESO_calc(ESO_t *ESO,int16_t val)
{
		//ESO补偿
		ESO->e = ESO->last_z1 - ESO->y;
		ESO->z1 = ESO->last_z1 + ESO->h * (ESO->last_z2 - ESO->deta1 * ESO->e);
		ESO->z2 = ESO->last_z2 + ESO->h * (ESO->last_z3 - ESO->deta2 * ESO->e + (ESO->K * ESO->Km)/(ESO->J * ESO->R) * ESO->u - (ESO->Km * ESO->Ke)/(ESO->J * ESO->R) * ESO->last_z2);
		ESO->z3 = ESO->last_z3 - ESO->h * ESO->deta3 * ESO->e;
		ESO->u = (val - ESO->last_z3) / ESO->b;
		ESO->last_z1 = ESO->z1;
		ESO->last_z2 = ESO->z2;
		ESO->last_z3 = ESO->z3;
		return 0;
}

int sgn(fp32 x)
{
	int y;
	if (x > 0)
		y = 1;
	else if (x < 0)
		y = -1;
	else
		y = 0;
	return y;
}

fp32 fhan(fp32 x1, fp32 x2, fp32 r, fp32 h)
{
	fp32 d, d0, y, a0, a, fst;
	d = r * h;
	d0 = d * h;
	y = x1 + h * x2;
	a0 = sqrt((d * d + 8 * r * ABS(y)));
	if (ABS(y) > d0)
		a = x2 + (a0 - d) / 2 * sgn(y);
	else
		a = x2 + y / h;
	if (ABS(a) > d)
		fst = -(r * sgn(a));
	else
		fst = -(r * a / d);
	return fst;
}

fp32 TD(fp32 v,fp32 r,fp32 h,fp32 h0)		//v 跟踪目标	r 跟踪速度（越大跟踪速度越快）	h 采样周期	h0 滤波因子（一般0.001-0.1，越小滤波效果越好）
{
	fp32 v1,v2,last_v1,last_v2;
	v1 = last_v1 + h * last_v2;
	v2 = last_v2 + h * fhan(last_v1 - v, last_v2, r, h0);
	last_v1 = v1;
	last_v2 = v2;
	return v2;
}

