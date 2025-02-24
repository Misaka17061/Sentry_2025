/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "imu_task.h"
#include "cmsis_os.h"
#include "sentry_def.h"
#include "IMU/imu_driver.h"
#include "bsp_init.h"
#include "pid.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/

/* ˽�к궨�� ----------------------------------------------------------------*/
#define IMU_TEMPERATURE    40.0f

#define TEMPERATURE_PID_KP 1600.0f
#define TEMPERATURE_PID_KI 0.2f
#define TEMPERATURE_PID_KD 0.0f

#define TEMPERATURE_PID_MAX_OUT  4500.0f
#define TEMPERATURE_PID_MAX_IOUT 4400.0f

#define TEMP_PWM_MAX 5000

/* ˽�б��� ------------------------------------------------------------------*/
osThreadId ImuTaskHandle;

/* imu_data */
IMU_Data_t* pimu;
/* PID�ṹ */
pid_t imu_temp_pid; // IMU�¶ȿ���PID

/* IMU���ȱ�� */
static uint8_t first_temperate = 0;

/* ��չ���� ------------------------------------------------------------------*/

/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void IMU_TempControl(fp32 temp);

/* ������ --------------------------------------------------------------------*/
void IMU_Task(void const*argument)
{
    uint32_t period = osKernelSysTick();

    for(;;)
    {
        IMU_Update((fp32)IMU_TASK_PERIOD / 1000.0f);
        IMU_TempControl(pimu->temp);
        osDelayUntil(&period, IMU_TASK_PERIOD);
    }
}

void IMU_TaskInit(void)
{
    IMU_Init();
    pimu = IMU_GetDataPointer();
    pid_init(&imu_temp_pid,
             POSITION_PID,
             TEMPERATURE_PID_MAX_OUT,
             TEMPERATURE_PID_MAX_IOUT,
             TEMPERATURE_PID_KP,
             TEMPERATURE_PID_KI,
             TEMPERATURE_PID_KD);

    osThreadDef(imu_task, IMU_Task, osPriorityNormal, 0, 256);
    ImuTaskHandle = osThreadCreate(osThread(imu_task), NULL);
}

/**
  * @brief ����BMI088���¶�
  */
static void IMU_TempControl(fp32 temp)
{
    uint16_t pwm;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        pid_calc(&imu_temp_pid, temp, IMU_TEMPERATURE);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        pwm = (uint16_t)imu_temp_pid.out;
        BSP_GPIO_SetPwmValue(&temp_pwm_gpio, pwm);
    }
    else
    {
        //in beginning, max power
        if (temp > IMU_TEMPERATURE)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.iout = TEMP_PWM_MAX / 2.0f;
            }
        }
        BSP_GPIO_SetPwmValue(&temp_pwm_gpio, TEMP_PWM_MAX - 1);
    }
}
