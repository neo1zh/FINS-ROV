//
// Created by mac on 2022/12/14.
//

#include "MyMotor.h"


//float Motor_4010::currentangle[4]={9000,0,9000,0};
uint8_t RxMessage[4][8];
/*

Motor_4015::Motor_4015(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) : CAN(commuInit), Motor(motorInit, this) {
    id = commuInit->_id - 0x141;
    ID_Bind_Rx(RxMessage);
}

Motor_4015::~Motor_4015() = default;

void Motor_4015::SetTargetAngle(float _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor_4015::CANMessageGenerate() {

    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = 0xA1;
        canQueue.Data[canQueue.rear].message[1] = 0;
        canQueue.Data[canQueue.rear].message[2] = 0;
        canQueue.Data[canQueue.rear].message[3] = 0;
        canQueue.Data[canQueue.rear].message[4] = motor4015_intensity[id];
        canQueue.Data[canQueue.rear].message[5] = motor4015_intensity[id] >> 8u;
        canQueue.Data[canQueue.rear].message[6] = 0;
        canQueue.Data[canQueue.rear].message[7] = 0;

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }

}


void Motor_4015::Handle() {
    int16_t intensity[8];
    uint8_t RxMessage[8];
    MotorStateUpdate();
    intensity[id] = IntensityCalc();
    Intensity[id] = intensity[id];
    if (stopFlag) {
        motor4015_intensity[id]= 0;
    } else {
        motor4015_intensity[id]= intensity[id];
    }

    CANMessageGenerate();
}

void Motor_4015::MotorStateUpdate() {

    feedback.angle = RxMessage[6] | (RxMessage[7] << 8u);
    feedback.speed = (int16_t) (RxMessage[4] | (RxMessage[5] << 8u));
    feedback.moment = (int16_t) (RxMessage[2] | (RxMessage[3] << 8u));
    feedback.temp = (int8_t) RxMessage[1];


    state.speed = (float) (feedback.speed) / reductionRatio;
    state.moment = feedback.moment;
    state.temperature = feedback.temp;
    state.angle = (float) (feedback.angle) * 360.0f / 16384.0f;
    Angle[id]=state.angle;

    thisAngle = feedback.angle;
    if (thisAngle <= lastRead) {
        if (lastRead - thisAngle > 8192)
            realAngle += (thisAngle + 16384.0f - lastRead) * 360.0f / 16384.0f / reductionRatio;
        else
            realAngle -= (lastRead - thisAngle) * 360.0f / 16384.0f / reductionRatio;
    } else {
        if (thisAngle - lastRead > 8192)
            realAngle -= (lastRead + 16384.0f - thisAngle) * 360.0f / 16384.0f / reductionRatio;
        else
            realAngle += (thisAngle - lastRead) * 360.0f / 16384.0f / reductionRatio;
    }
    state.angle = realAngle;
    lastRead = feedback.angle;

}

int16_t Motor_4015::IntensityCalc() {
    int16_t intensity = 0;
    float _targetSpeed = anglePID.PIDCalc(targetAngle, state.angle);
    intensity = (int16_t) speedPID.PIDCalc(_targetSpeed, state.speed);
	  //intensity = anglePID.PIDCalc(targetAngle, state.angle);
    return intensity;
}


*/


/*4010电机类------------------------------------------------------------------*/
/**
 * @brief Motor_4010类的构造函数
 */
 
FOUR_Motor_4010::FOUR_Motor_4010(COMMU_INIT_t *commu_init1, COMMU_INIT_t *commu_init2,
                                 COMMU_INIT_t *commu_init3, COMMU_INIT_t *commu_init4, MOTOR_INIT_t *motor_init1,
                                 MOTOR_INIT_t *motor_init2)
        : Motor(motor_init1, this) {
    canIDs[0] = commu_init1->_id;
    canIDs[1] = commu_init2->_id;
    canIDs[2] = commu_init3->_id;
    canIDs[3] = commu_init4->_id;
    if (motor_init1->speedPIDp) speedPIDs[0].PIDInfo = *motor_init1->speedPIDp;
    if (motor_init2->speedPIDp) speedPIDs[1].PIDInfo = *motor_init2->speedPIDp;
    if (motor_init1->speedPIDp) speedPIDs[2].PIDInfo = *motor_init1->speedPIDp;
    if (motor_init1->speedPIDp) speedPIDs[3].PIDInfo = *motor_init1->speedPIDp;

    if (motor_init1->anglePIDp) anglePIDs[0].PIDInfo = *motor_init1->anglePIDp;
    if (motor_init2->anglePIDp) anglePIDs[1].PIDInfo = *motor_init2->anglePIDp;
    if (motor_init1->anglePIDp) anglePIDs[2].PIDInfo = *motor_init1->anglePIDp;
    if (motor_init1->anglePIDp) anglePIDs[3].PIDInfo = *motor_init1->anglePIDp;
    FOURID_Bind_Rx(canIDs, RxMessage);

}

/**
 * @brief Motor_4010类的析构函数
 */
 
FOUR_Motor_4010::~FOUR_Motor_4010() = default;

/**
 * @brief 4010电机类的执行处理函数
 */

void FOUR_Motor_4010::Handle() {
    int16_t intensity[4];
    uint32_t id;

    for (auto canID: canIDs) {
        id = canID - 0x141;
        MotorStateUpdate(id);
        intensity[id] = IntensityCalc(id);
        if (stopFlag) {
            motor4010_intensity[id] = 0;
        } else {
            motor4010_intensity[id] = intensity[id];
        }
    }
    CANMessageGenerate();
}

/**
 * @brief 4010电机类的消息包获取任务
 */

void FOUR_Motor_4010::CANMessageGenerate() {
    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = motor4010_intensity[0];
        canQueue.Data[canQueue.rear].message[1] = motor4010_intensity[0] >> 8u;
        canQueue.Data[canQueue.rear].message[2] = motor4010_intensity[1];
        canQueue.Data[canQueue.rear].message[3] = motor4010_intensity[1] >> 8u;
        canQueue.Data[canQueue.rear].message[4] = motor4010_intensity[2];
        canQueue.Data[canQueue.rear].message[5] = motor4010_intensity[2] >> 8u;
        canQueue.Data[canQueue.rear].message[6] = motor4010_intensity[3];
        canQueue.Data[canQueue.rear].message[7] = motor4010_intensity[3] >> 8u;

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }
}

/**
 * @brief 用于设置4010电机速度
 * @param _targetSpeed 目标速度
 */

void FOUR_Motor_4010::SetTargetSpeed(const float *_targetSpeed) {
    stopFlag = false;
    targetSpeed[0] = _targetSpeed[0];
    targetSpeed[1] = _targetSpeed[1];
    targetSpeed[2] = _targetSpeed[2];
    targetSpeed[3] = _targetSpeed[3];

}

void FOUR_Motor_4010::SetTargetAngle(const float *_targetAngle) {
    stopFlag = false;
    targetAngle[0] = _targetAngle[0];
    targetAngle[1] = _targetAngle[1];
    targetAngle[2] = _targetAngle[2];
    targetAngle[3] = _targetAngle[3];

}


/**
 * @brief 更新电机的相关状态
 * @callergraph this->Handle()
 */

void FOUR_Motor_4010::MotorStateUpdate(uint32_t id) {
    feedback[id].angle = RxMessage[id][6] | (RxMessage[id][7] << 8u);
    feedback[id].speed = RxMessage[id][4] | (RxMessage[id][5] << 8u);
    feedback[id].moment = RxMessage[id][2] | (RxMessage[id][3] << 8u);
    feedback[id].temp = RxMessage[id][1];

    switch (ctrlType) {
        case SPEED_Single: {
            state[id].speed = feedback[id].speed / reductionRatio;
        }
        case POSITION_Double: {
            state[id].speed = feedback[id].speed / reductionRatio;
            state[id].moment = feedback[id].moment;
            state[id].temperature = feedback[id].temp;
            state[id].angle = feedback[id].angle * 360 / 16384;
           // realAngle = state[id].angle;
            thisAngle = feedback[id].angle;
            static int32_t lastRead = 0;
            if (thisAngle <= lastRead) {
                if (lastRead - thisAngle > 8000)
                    realAngle += (thisAngle + 16384 - lastRead) * 360.0f / 16384.0f / reductionRatio;
                else
                    realAngle -= (lastRead - thisAngle) * 360.0f / 16384.0f / reductionRatio;
            } else {
                if (thisAngle - lastRead > 8000)
                    realAngle -= (lastRead + 16384 - thisAngle) * 360.0f / 16384.0f / reductionRatio;
                else
                    realAngle += (thisAngle - lastRead) * 360.0f / 16384.0f / reductionRatio;
            }
            state[id].angle = realAngle;
            lastRead = feedback[id].angle;
            break;
        }
        case DIRECT:

            break;
    }

}

/**
 * @brief 计算电机实际控制电流
 * @return 控制电流值
 */

int16_t FOUR_Motor_4010::IntensityCalc(uint32_t id) {
    int16_t intensity = 0;

    switch (ctrlType) {
        case DIRECT:
            intensity = targetSpeed[id] * 16384 / 360.0f;
            break;

        case SPEED_Single:
            intensity = speedPIDs[id].PIDCalc(targetSpeed[id], state[id].speed);
            break;

        case POSITION_Double:
            float _targetSpeed = anglePIDs[id].PIDCalc(targetAngle[id], state[id].angle);
            intensity = (int16_t) speedPIDs[id].PIDCalc(_targetSpeed, state[id].speed);
            break;
    }
    return intensity;
};




Motor_4010::Motor_4010(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit) : CAN(commuInit), Motor(motorInit, this) {
    id = commuInit->_id - 0x141;
    ID_Bind_Rx(RxMessage);

}

Motor_4010::~Motor_4010() = default;


void Motor_4010::SetTargetAngle(int32_t _targetAngle) {
    stopFlag = false;
    targetAngle = _targetAngle;
}

void Motor_4010::SetTargetSpeed(float _targetSpeed) {
    stopFlag = false;
    targetSpeed = _targetSpeed;
}

void Motor_4010::CANMessageGenerate() {
    if ((canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT != canQueue.front) {

        canQueue.Data[canQueue.rear].ID = can_ID;
        canQueue.Data[canQueue.rear].canType = canType;
        canQueue.Data[canQueue.rear].message[0] = 0xA4;
        canQueue.Data[canQueue.rear].message[1] = 0x00;
        canQueue.Data[canQueue.rear].message[2] = *(uint8_t *)(&Motor4010_Maxspeed);
        canQueue.Data[canQueue.rear].message[3] = *((uint8_t *)(&Motor4010_Maxspeed)+1);
        canQueue.Data[canQueue.rear].message[4] = *Motor4010_Angle;
        canQueue.Data[canQueue.rear].message[5] = *(Motor4010_Angle+1);
        canQueue.Data[canQueue.rear].message[6] = *(Motor4010_Angle+2);
        canQueue.Data[canQueue.rear].message[7] = *(Motor4010_Angle+3);

        canQueue.rear = (canQueue.rear + 1) % canQueue.MAX_MESSAGE_COUNT;
    }
}

void Motor_4010::Handle() {


    feedback.angle = (int16_t)(RxMessage[6] | (RxMessage[7] << 8u));
    feedback.speed = (int16_t)(RxMessage[4] | (RxMessage[5] << 8u));
    feedback.moment = (int16_t)(RxMessage[2] | (RxMessage[3] << 8u));
    feedback.temp = (int8_t) RxMessage[1];
    state.angle = (float) (feedback.angle) * 360.0f / 16384.0f;

    if (stopFlag) {
        Motor4010_Angle = 0;
    } else {
        Motor4010_Angle = (uint8_t *) &targetAngle;
        Motor4010_Maxspeed = 0x60;
    }
    CANMessageGenerate();

}