//
// Created by admin on 2023/10/7.
//
#include "Device.h"
#include "B02_middleware.h"
#ifndef CONTROL_FRAME_MAIN_MYIIC_H
#define CONTROL_FRAME_MAIN_MYIIC_H

/////////////////////////////////////////////////////////////////////////////////////////
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////

//IO��������,
#define SDA_IN()  {GPIOF->CRL&=0XFFFFFF0F;GPIOF->CRL|=8<<4;} // PF1 = SDA ��Ϊ����ʱ
#define SDA_OUT() {GPIOF->CRL&=0XFFFFFF0F;GPIOF->CRL|=3<<4;} // PF1 = SDA ��Ϊ���ʱ

//IO��������
#define B02_Port GPIOF
#define B02_SCL_PIN    GPIO_PIN_1 //SCL = PF0
#define B02_SDA_PIN    GPIO_PIN_0 //SDA = PF1 ��Ϊ���ʱ
#define READ_SDA   PFin(1)  //SDA = PF1 ��Ϊ����ʱ

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��
void IIC_Start(void);								//����IIC��ʼ�ź�
void IIC_Stop(void);	  						//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void IIC_Ack(void);									//IIC����ACK�ź�
void IIC_NAck(void);								//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);



#endif //CONTROL_FRAME_MAIN_MYIIC_H
