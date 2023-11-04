//
// Created by admin on 2023/10/7.
//

#ifndef CONTROL_FRAME_MAIN_B02_MIDDLEWARE_H
#define CONTROL_FRAME_MAIN_B02_MIDDLEWARE_H

#include "main.h"

#define B02_IIC_ADDRESS (0xEC)  //IST8310��IIC��ַ
#define B02_IIC_READ_MSB (0x80) //IST8310��SPI��ȡ���͵�һ��bitΪ1

extern void B02_GPIO_init(void); //ist8310��io��ʼ��
extern void B02_com_init(void);  //ist8310��ͨѶ��ʼ��
extern uint8_t B02_IIC_read_single_reg(uint8_t reg);
extern void B02_IIC_write_single_reg(uint8_t reg, uint8_t data);
extern void B02_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void B02_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern void B02_delay_ms(uint16_t ms);
extern void B02_delay_us(uint16_t us);
extern void B02_RST_H(void); //��λIO �ø�
extern void B02_RST_L(void); //��λIO �õ� �õػ�����ist8310����





#endif //CONTROL_FRAME_MAIN_B02_MIDDLEWARE_H
