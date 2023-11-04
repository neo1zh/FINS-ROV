//
// Created by 25396 on 2023/2/11.
//

#ifndef COMMUTYPE_H
#define COMMUTYPE_H

#include "Device.h"
#include "Map.h"


#define can1 1
#define can2 2

/*结构体定义--------------------------------------------------------------*/

typedef struct {
    uint32_t _id;//canID
    int canType;
} COMMU_INIT_t;
typedef struct {
    uint32_t ID;
    int canType;
    uint8_t message[8];
}DATA_t;
typedef struct {
    DATA_t Data[8];
    int front;
    int rear;
    const int MAX_MESSAGE_COUNT;
}TX_QUEUE_t;

/*类型定义----------------------------------------------------------------*/
/*CAN类------------------------------------------------------------------*/
class CAN {
public:
    uint32_t can_ID;
    static TX_QUEUE_t canQueue;
    static void CANInit();

    CAN();

    explicit CAN(COMMU_INIT_t *_init);

    ~CAN();

    static void CANPackageSend();

    static void Rx_Handle(CAN_HandleTypeDef *hcan);

    virtual void CANMessageGenerate() = 0;

protected:
    int canType;
    static MyMap<uint32_t, uint8_t *> dict;

    void ID_Bind_Rx(uint8_t *RxMessage) const;

    void FOURID_Bind_Rx(uint32_t *canID, uint8_t (*RxMessage)[8]);
};

#endif //MYMOYOR1_COMMUTYPE_H
