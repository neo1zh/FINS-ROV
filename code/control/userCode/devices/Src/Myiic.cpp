#include "myiic.h"
/////////////////////////////////////////////////////////////////////////////////////////
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////


//����IIC��ʼ�ź�
void IIC_Start(void)
{
    SDA_OUT();     //sda�����
    HAL_GPIO_WritePin(B02_Port, B02_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_SET);
    B02_delay_us(5);
    HAL_GPIO_WritePin(B02_Port, B02_SDA_PIN, GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low
    B02_delay_us(5);
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_RESET);//ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void IIC_Stop(void)
{
    SDA_OUT();//sda�����
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B02_Port, B02_SDA_PIN, GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
    B02_delay_us(5);
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(B02_Port, B02_SDA_PIN, GPIO_PIN_SET);//����I2C���߽����ź�
    B02_delay_us(5);
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    SDA_IN();      //SDA����Ϊ����
    HAL_GPIO_WritePin(B02_Port, B02_SDA_PIN, GPIO_PIN_SET);B02_delay_us(5);
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_SET);B02_delay_us(5);
    while(HAL_GPIO_ReadPin(B02_Port, B02_SDA_PIN)==GPIO_PIN_SET)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_RESET);//ʱ�����0
    return 0;
}
//����ACKӦ��
void IIC_Ack(void)
{
    HAL_GPIO_WritePin(B02_Port, B02_SCL_PIN, GPIO_PIN_RESET);
    SDA_OUT();
    HAL_GPIO_WritePin(B02_Port, B02_SDA_PIN, GPIO_PIN_RESET);
    delay_us(5);
    IIC_SCL=1;
    delay_us(5);
    IIC_SCL=0;
}
//������ACKӦ��
void IIC_NAck(void)
{
    IIC_SCL=0;
    SDA_OUT();
    IIC_SDA=1;
    delay_us(5);
    IIC_SCL=1;
    delay_us(5);
    IIC_SCL=0;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
        delay_us(5);
        IIC_SCL=1;
        delay_us(5);
        IIC_SCL=0;
        delay_us(5);
    }
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
    {
        IIC_SCL=0;
        delay_us(5);
        IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;
        delay_us(5);
    }
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK
    return receive;
}

//
// Created by admin on 2023/10/7.
//
