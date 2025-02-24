#ifndef ESO_H
#define ESO_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "struct_typedef.h"
#include "math.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
typedef struct
{
	fp32 h;
	fp32 K;
	fp32 Km;
	fp32 Ke;
	fp32 J;
	fp32 R;
	fp32 b;
	fp32 w;
	fp32 y;
	fp32 last_y;
	fp32 delta_v;
	fp32 e,z1,z2,z3,last_z1,last_z2,last_z3,u;
	fp32 deta1,deta2,deta3;
}ESO_t;

/* �궨�� --------------------------------------------------------------------*/
#define ABS(x) (((x) > 0) ? (x) : (-(x)))
/* ��չ���� ------------------------------------------------------------------*/
extern ESO_t ESO_6020;
extern ESO_t ESO_9025;
/* �������� ------------------------------------------------------------------*/
void ESO_6020_init(void);
void ESO_9025_init(void);
int32_t ESO_calc(ESO_t *ESO, int16_t val);
int sgn(fp32 x);
#endif  
