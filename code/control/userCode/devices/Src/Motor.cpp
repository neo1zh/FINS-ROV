//
// Created by LEGION on 2021/10/4.
//

#include "Motor.h"


Motor_Object_t *Motor::head_;

/*Motor类----------------------------------------------------------------*/
/**
 * @brief Motor类的构造函数
 * @param _init 类的初始化结构体指针
 */
Motor::Motor(MOTOR_INIT_t *_init, Motor *motor) {
    deviceType = MOTOR;

    if (_init->speedPIDp) speedPID.PIDInfo = *_init->speedPIDp;
    if (_init->anglePIDp) anglePID.PIDInfo = *_init->anglePIDp;
    ctrlType = _init->ctrlType;
    reductionRatio = _init->reductionRatio;

    auto *new_object = new Motor_Object_t();
    new_object->motor_object = motor;
    new_object->next = head_;
    head_ = new_object;

}

/**
* @brief Motor类的析构函数
*/
Motor::~Motor() = default;

void Motor::ErrorHandle() {}

void Motor::MotorsHandle() {

    Motor_Object_t *current = head_;
    while (current) {
        current->motor_object->Handle();
        current = current->next;
    }

}

void Motor::Stop() {
    stopFlag = true;

}






