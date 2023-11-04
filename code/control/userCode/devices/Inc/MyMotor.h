//
// Created by mac on 2022/12/14.
//

#ifndef MYMOTOR_H
#define MYMOTOR_H

#include "Device.h"
#include "can.h"
#include "Motor.h"
#include "CommuType.h"



class Motor_4015 : public Motor, public CAN {
    public:
        uint8_t RxMessage[8]{};
        int16_t motor4015_intensity[8]{};

        void CANMessageGenerate() override;

        void Handle() override;

        void SetTargetAngle(float _targetAngle);

        Motor_4015(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

        ~Motor_4015();

    private:
        uint8_t id;
        C6x0Rx_t feedback{};
        float targetAngle{};
        MOTOR_STATE_t state{};
        float realAngle{0};
        float thisAngle;
        float lastRead;
        static int16_t Intensity[8];
        static float Angle[8];

        void MotorStateUpdate();

        int16_t IntensityCalc();
    };



class FOUR_Motor_4010 : public Motor, public CAN {
public:
    uint8_t RxMessage[4][8]{};
    int16_t motor4010_intensity[4]{};

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetSpeed(const float *_targetSpeed);

    void SetTargetAngle(const float *_targetAngle);

    FOUR_Motor_4010(COMMU_INIT_t *commu_init1, COMMU_INIT_t *commu_init2,
                    COMMU_INIT_t *commu_init3, COMMU_INIT_t *commu_init4, MOTOR_INIT_t *motor_init1,
                    MOTOR_INIT_t *motor_init2);

    ~FOUR_Motor_4010();

private:
    float realAngle;
    float thisAngle;
    uint32_t canIDs[4]{};
    PID speedPIDs[4];
    PID anglePIDs[4];
    C6x0Rx_t feedback[4]{};
    float targetSpeed[4]{};
    float targetAngle[4]{};
    MOTOR_STATE_t state[4]{};
    static int16_t Intensity[8];
    void MotorStateUpdate(uint32_t id);

    int16_t IntensityCalc(uint32_t id);
};

class Motor_4010 : public Motor, public CAN {
public:
    uint8_t* Motor4010_Angle;
    uint16_t Motor4010_Maxspeed;
    //uint8_t* Motor4010_Direction;
    uint32_t sendSpeed{};
    uint8_t RxMessage[8]{};
    MOTOR_STATE_t state{};
    static void Init();

    void CANMessageGenerate() override;

    void Handle() override;

    void SetTargetAngle(int32_t _targetAngle);

    void SetTargetSpeed(float _targetSpeed);

    Motor_4010(COMMU_INIT_t *commuInit, MOTOR_INIT_t *motorInit);

    ~Motor_4010();

private:
    uint8_t id;
    C6x0Rx_t feedback{};

    int32_t targetAngle{};
    float targetSpeed{};
    float spinDirection{};
    //static float currentangle[4];
};
#endif //MYMOYOR1_MYMOTOR_H
