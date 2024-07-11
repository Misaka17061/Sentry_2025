/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "comm_task.h"
#include "cmsis_os.h"
#include "sentry_def.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/

/* ˽�б��� ------------------------------------------------------------------*/
osThreadId CommTaskHandle;

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/

/* ������ --------------------------------------------------------------------*/
void Comm_Task(void const*argument)
{
    for(;;)
    {
        Comm_ReceiveDataHandler();//���������ڸ����սṹ�建����������
        Comm_TransmitDataHandler();
        osDelay(COMM_TASK_PERIOD);
    }
}

void Comm_TaskInit(void)
{
    osThreadDef(comm_task, Comm_Task, osPriorityNormal, 0, 256);
    CommTaskHandle = osThreadCreate(osThread(comm_task), NULL);
}
