//
// Created by LEGION on 2021/10/4.
//

#ifndef MYMOTOR1_ANGLECTRL_H
#define MYMOTOR1_ANGLECTRL_H

#include "main.h"
#include "MyMotor.h"
#include "Device.h"

/*枚举类型定义------------------------------------------------------------*/

/*
 * 底盘任务错误信息枚举
 */
typedef enum {
    NONE
} CHASSIS_ERROR_E;

/*结构体定义--------------------------------------------------------------*/


/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
extern float RLInputAngle;
extern float UDInputAngle;
/*外部函数声明-------------------------------------------------------------*/
void anglectrl_1();

void anglectrl_motor();

void anglectrl_servo();

void speedctrl_propeller();

void SerialRx_2();

void SerialRx_4();

void anglectrl_servo_2();
#endif //MYMOTOR1_ANGLECTRL_H