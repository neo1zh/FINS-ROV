/////////////////////////////////////////////////////////////////////////////////////////
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////
#ifndef __MS5837_H_
#define __MS5837_H_

#include "B02_middleware.h"
#include "usart.h"
#include "myiic.h"


/* �������� -----------------------------------------------*/



#define MS5837_30BA_WriteCommand     0xEC
#define MS5837_30BA_ReadCommand      0xED

#define MS5837_30BA_ResetCommand     0x1E                //��λ
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM��ȡ,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC��ȡ

#define MS5837_30BA_OSR256					 0x40
#define MS5837_30BA_OSR512					 0x42
#define MS5837_30BA_OSR1024					 0x44
#define MS5837_30BA_OSR2048					 0x46
#define MS5837_30BA_OSR4096					 0x48
#define	MS5837_30BA_D2_OSR_8192   	 0x58                 //16.44msת��ʱ��
#define	MS5837_30BA_D1_OSR_8192   	 0x48                 //16.44msת��ʱ��

u8   MS5837_30BA_PROM(void);
void MS5837_30BA_GetData(void);
void MS5837_30BA_ReSet(void);
unsigned char MS5837_30BA_Crc4(void);
unsigned long MS5837_30BA_GetConversion(u8 command);
extern double Temperature;
extern signed int dT,TEMP;
extern int32_t Pressure;
extern uint32_t Cal_C[7];

#endif//
// Created by admin on 2023/10/7.
//

#ifndef CONTROL_FRAME_MAIN_MS5837_H
#define CONTROL_FRAME_MAIN_MS5837_H

#endif //CONTROL_FRAME_MAIN_MS5837_H
