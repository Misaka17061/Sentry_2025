#ifndef	TIMER_TASK_H
#define TIMER_TASK_H

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "soft_timer.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
typedef int32_t (*SoftTimer_Callback_t)(void *argc);

typedef struct
{
    uint8_t id;
    uint32_t ticks;
    void *argc;
    SoftTimer_Callback_t callback;
} SoftwareTimer_t;

/* �궨�� --------------------------------------------------------------------*/

/* ��չ���� ------------------------------------------------------------------*/

/* �������� ------------------------------------------------------------------*/
void SoftwareTimerTaskInit(void);
int32_t SoftwareTimerRegister(SoftTimer_Callback_t callback_t, void *argc, uint32_t ticks);

#endif
