//
// Created by LEGION on 2021/10/4.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "Device.h"
#include "can.h"
#include "PID.h"
#include "CommuType.h"
#include <cstring>

/*枚举类型定义------------------------------------------------------------*/
/**
 * @enum 控制电机的方式
 * @example SPEED_Single 单环电机，控制速度
 * @example POSITION_Double 双环电机，控制角度
 */

/*结构体定义--------------------------------------------------------------*/
typedef enum {
    DIRECT = 0,
    SPEED_Single,
    POSITION_Double
} MOTOR_CTRL_TYPE_e;

typedef struct {
    uint16_t angle;
    int16_t speed;
    int16_t moment;
    int8_t temp;
} C6x0Rx_t;

typedef struct {
    PID_Regulator_t *speedPIDp;//速度环pid参数结构体指针
    PID_Regulator_t *anglePIDp;//角度环pid参数结构体指针
    MOTOR_CTRL_TYPE_e ctrlType;
    float reductionRatio;//减速比
} MOTOR_INIT_t;


typedef struct {

    float speed;//最终电机输出轴的转速，单位为RPM
    float angle;//输出轴的角度，单位为度
    float moment;//转矩电流的相对值，具体值参考电调手册
    float temperature;//电机温度，单位摄氏度

} MOTOR_STATE_t;

class Motor;

struct Motor_Object_t {
    Motor *motor_object;
    Motor_Object_t *next;
};

/*类型定义----------------------------------------------------------------*/

/*Motor类----------------------------------------------------------------*/
class Motor : private Device {
public:
    Motor(MOTOR_INIT_t *_init, Motor *motor);

    ~Motor();

    void ErrorHandle() override;

    void Stop();

    static void MotorsHandle();
protected:
    PID speedPID, anglePID;
    float reductionRatio;
    bool stopFlag;
    MOTOR_CTRL_TYPE_e ctrlType;
private:
    static Motor_Object_t *head_;

};

/*结构体成员取值定义组------------------------------------------------------*/

/**
 * @defgroup motor_IDs
 * @brief 电机ID前八个对应C型开发板can1上1到8的ID，9到16对应C型开发板can2上的1到8
 */
#define MOTOR_ID_1 0
#define MOTOR_ID_2 1
#define MOTOR_ID_3 2
#define MOTOR_ID_4 3
#define MOTOR_ID_5 4
#define MOTOR_ID_6 5
#define MOTOR_ID_7 6
#define MOTOR_ID_8 7

/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //MYMOYOR1_MOTOR_H
