#include "ist8310driver_middleware.h"
#include "main.h"
#include "bsp_delay.h"

extern I2C_HandleTypeDef hi2c3;


void ist8310_GPIO_init(void)
{
}

void ist8310_com_init(void)
{
}


uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS<<1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10);
    return res;
}
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS<<1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);

}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS<<1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS<<1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10);
}
void ist8310_delay_ms(uint16_t ms)
{
    BSP_DelayMs(ms);
}
void ist8310_delay_us(uint16_t us)
{
    BSP_DelayMs(us);
}

void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
}
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
}
