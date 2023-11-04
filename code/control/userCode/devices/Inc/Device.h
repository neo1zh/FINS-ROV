//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_DEVICE_H
#define RM_FRAME_C_DEVICE_H
#include "main.h"

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "usbd_cdc_if.h"

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
/*枚举类型定义------------------------------------------------------------*/

//extern void ChassisStart();
//extern void ChassisHandle();
//extern void CtrlHandle();
//extern void ARMHandle();

/*
 * 设备类型枚举
 */
typedef enum {
    MOTOR,
    SERVO,
} DEVICE_TYPE_E;

const int length=50;//串口通信长度，若使用串口调试器，2组数据为15，4组数据为29，若使用python程序发送，2组数据为13，4组数据为27.
extern uint8_t RxBuffer[length];//串口接收缓冲区，接收到的数据存在这里
extern uint8_t RxBuffer2[11];
extern int32_t Angle[6];//根据接收到的数据计算得出，为角度值乘100，初始值为9000（90度）
extern int32_t Newdata[6];
extern int32_t sonar;
extern int counter;
extern float currentangle[4];


extern void anglectrl_1();

extern void anglectrl_motor();

extern void anglectrl_servo();

extern void speedctrl_propeller();

extern void anglectrl_servo_2();

extern void SerialRx_2();

extern void SerialRx_4();

extern void SerialRx_6();

extern void Newdatahandle(uint8_t *rx_buffer, int32_t *pwm_value);




/*结构体定义--------------------------------------------------------------*/
typedef struct {
    uint32_t robot_ID;
    uint32_t yaw_zero;
    uint32_t pitch_zero;
    uint32_t pat[125];
}flash_data_t;

typedef struct {
    float x,y,z;
    float temp;
}ist8310_t;
/*类型定义----------------------------------------------------------------*/

class Device {
protected:
    DEVICE_TYPE_E deviceType;
    uint32_t deviceID;

    virtual void Handle() = 0;

    virtual void ErrorHandle() = 0;

};

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_DEVICE_H
