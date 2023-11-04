#include "B02_middleware.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c3;


void B02_GPIO_init(void)
{
}

void B02_com_init(void)
{
}


uint8_t B02_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c2, B02_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}
void B02_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, B02_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

}
void B02_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c2, B02_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
void B02_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
void B02_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}
void B02_delay_us(uint16_t us)
{
    uint16_t i;
    for (i = 0; i < us; i++ )
    {
        int a = 10;  //delay based on mian clock, 168Mhz
        while (a-- );
    }
}

//
// Created by admin on 2023/10/7.
//
