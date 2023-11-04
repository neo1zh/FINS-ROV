//
// Created by LEGION on 2021/10/4.
//
#include "Anglectrl.h"

/*float RLInputAngle=0;
float UDInputAngle=0;

PID_Regulator_t pidRegulator1 = {
        .kp = 0.155,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 500
};
PID_Regulator_t pidRegulator2 = {
        .kp = 28,
        .ki = 0.3,
        .kd = 10,
        .componentKpMax = 500,
        .componentKiMax = 500,
        .componentKdMax = 500,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};

PID_Regulator_t pidRegulator3 = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 500
};
PID_Regulator_t pidRegulator4 = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 2000,
        .componentKiMax = 500,
        .componentKdMax = 500,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t RLMotorInit = {
        .speedPIDp = &pidRegulator1,
        .anglePIDp = &pidRegulator2,
        .ctrlType = POSITION_Double,
        .reductionRatio = 1.0f
};

COMMU_INIT_t RLCommuInit = {
        ._id = 0x142,
        .canType = can1
};

MOTOR_INIT_t UDMotorInit = {
        .speedPIDp = &pidRegulator3,
        .anglePIDp = &pidRegulator4,
        .ctrlType = POSITION_Double,
        .reductionRatio = 1.0f
};

COMMU_INIT_t UDCommuInit = {
        ._id = 0x141,
        .canType = can1
};

Motor_4015 RLMotor(&RLCommuInit, &RLMotorInit);
Motor_4015 UDMotor(&UDCommuInit, &UDMotorInit);

void anglectrl_1(){
    if(RxBuffer[0]=='-') RLInputAngle=45.46-10*(RxBuffer[1]-48)-(RxBuffer[2]-48);
    else RLInputAngle=16.5+10*(RxBuffer[1]-48)+(RxBuffer[2]-48);
    if(RxBuffer[3]=='-') UDInputAngle=240-10*(RxBuffer[4]-48)-(RxBuffer[5]-48);
    else UDInputAngle=16+10*(RxBuffer[4]-48)+(RxBuffer[5]-48);
    RLMotor.SetTargetAngle(RLInputAngle);
    UDMotor.SetTargetAngle(UDInputAngle);
}*/



PID_Regulator_t pidRegulator1 = {//电机1 pid速度环
        .kp = 0.17,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 500,
        .componentKiMax = 500,
        .componentKdMax = 0,
        .outputMax = 2000        //4010电机输出电流上限，可以调小，勿调大
};
PID_Regulator_t pidRegulator2 = {//电机1 pid角度环
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 500,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 
};
PID_Regulator_t pidRegulator3 = {//电机2 pid速度环
        .kp = -0.45,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 500,
        .componentKiMax = 500,
        .componentKdMax = 0,
        .outputMax = 2000
};
PID_Regulator_t pidRegulator4 = {//电机2 pid角度环
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 500,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 2000 //4010电机输出电流上限，可以调小，勿调大
};
MOTOR_INIT_t chassisMotorInit1 = {
        .speedPIDp = &pidRegulator1,
        .anglePIDp = &pidRegulator2,
				.ctrlType = POSITION_Double,
        .reductionRatio = 1.0f
};
MOTOR_INIT_t chassisMotorInit2 = {
        .speedPIDp = &pidRegulator3,
        .anglePIDp = &pidRegulator4,
				.ctrlType = POSITION_Double,
        .reductionRatio = 1.0f
};

MOTOR_INIT_t chassisMotorInit3 = {
        .speedPIDp = nullptr,
        .anglePIDp = nullptr,
				.ctrlType = DIRECT,
        .reductionRatio = 1.0f
};

COMMU_INIT_t chassisCommuInit1 = {
        ._id = 0x141,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit2 = {
        ._id = 0x142,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit3 = {
        ._id = 0x143,
        .canType = can1
};
COMMU_INIT_t chassisCommuInit4 = {
        ._id = 0x144,
        .canType = can1
};

// Classis_Motor(&chassisCommuInit1, &chassisCommuInit2, &chassisCommuInit3, &chassisCommuInit4,
   //                           &chassisMotorInit1, &chassisMotorInit2);

Motor_4010 Motor0(&chassisCommuInit1, &chassisMotorInit3);
Motor_4010 Motor1(&chassisCommuInit2, &chassisMotorInit3);
Motor_4010 Motor2(&chassisCommuInit3, &chassisMotorInit3);
Motor_4010 Motor3(&chassisCommuInit4, &chassisMotorInit3);
/*Motor0: 补光灯支架上方电机
Motor1: 相机支架上方电机
Motor2: 补光灯支架下方电机
Motor3: 相机支架下方电机
 */
void anglectrl_motor(){//控制电机
    //currentangle[0]=Motor1.state.angle;
    //currentangle[1]=Motor2.state.angle;
    //currentangle[2]=Motor3.state.angle;
    //currentangle[3]=Motor4.state.angle;

    Motor0.SetTargetAngle(Angle[0]);
    Motor1.SetTargetAngle(Angle[1]);
    Motor2.SetTargetAngle(Angle[2]);
    Motor3.SetTargetAngle(Angle[3]);
}
void anglectrl_servo(){//控制舵机

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Angle[0]/9.0+500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Angle[2]/9.0+500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Angle[1]/9.0+500);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Angle[3]/9.0+500);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, Angle[4]/9.0+500);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, Angle[5]/9.0+500);
}

void speedctrl_propeller(){//控制推进器
    if(counter<250){
        counter++;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1500);
	      __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);
    }
    else speedctrl_propeller();
}

void speedctrl_propeller(){//控制推进器
    if(counter<250){
        counter++;
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1500);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1500);
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);
    }
    else{
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Newdata[0]);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Newdata[1]);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Newdata[2]);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Newdata[3]);
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, Newdata[4]);
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, Newdata[5]);
    }
}

void SerialRx_2(){//接收2组数据
    Angle[0]=(RxBuffer[1]-'0')*10000+(RxBuffer[2]-'0')*1000+(RxBuffer[3]-'0')*100+(RxBuffer[4]-'0')*10+(RxBuffer[5]-'0');
    Angle[2]=(RxBuffer[8]-'0')*10000+(RxBuffer[9]-'0')*1000+(RxBuffer[10]-'0')*100+(RxBuffer[11]-'0')*10+(RxBuffer[12]-'0');


    if(Angle[0]>4500) Angle[0]=4500;
    if(Angle[2]>4500) Angle[2]=4500;
	
    if(RxBuffer[0]=='0') Angle[0]=9000+Angle[0];
    else Angle[0]=9000-Angle[0];
    if(RxBuffer[7]=='0') Angle[2]=9000+Angle[2];
    else Angle[2]=9000-Angle[2];


    //HAL_UART_Transmit_IT(&huart6,RxBuffer,length);
    HAL_UART_Receive_IT(&huart6,RxBuffer,length);
}

/*void SerialRx_4(){//接收4组数据
    Angle[0]=(RxBuffer[1]-'0')*10000+(RxBuffer[2]-'0')*1000+(RxBuffer[3]-'0')*100+(RxBuffer[4]-'0')*10+(RxBuffer[5]-'0');
    Angle[2]=(RxBuffer[8]-'0')*10000+(RxBuffer[9]-'0')*1000+(RxBuffer[10]-'0')*100+(RxBuffer[11]-'0')*10+(RxBuffer[12]-'0');
    Angle[1]=(RxBuffer[15]-'0')*10000+(RxBuffer[16]-'0')*1000+(RxBuffer[17]-'0')*100+(RxBuffer[18]-'0')*10+(RxBuffer[19]-'0');
    Angle[3]=(RxBuffer[22]-'0')*10000+(RxBuffer[23]-'0')*1000+(RxBuffer[24]-'0')*100+(RxBuffer[25]-'0')*10+(RxBuffer[26]-'0');

    if(Angle[0]>4500) Angle[0]=4500;
    if(Angle[2]>4500) Angle[2]=4500;
    if(RxBuffer[0]=='0') Angle[0]=9000+Angle[0];
    else Angle[0]=9000-Angle[0];
    if(RxBuffer[7]=='0') Angle[2]=9000+Angle[2];
    else Angle[2]=9000-Angle[2];

    if(Angle[1]>4500) Angle[1]=4500;
    if(Angle[3]>4500) Angle[3]=4500;
    if(RxBuffer[14]=='0') Angle[1]=9000+Angle[1];
    else Angle[1]=9000-Angle[1];
    if(RxBuffer[21]=='0') Angle[3]=9000+Angle[3];
    else Angle[3]=9000-Angle[3];
    //HAL_UART_Transmit_IT(&huart6,RxBuffer,length);
    HAL_UART_Receive_IT(&huart6,RxBuffer,length);
}*/

void SerialRx_6(){
    Angle[0]=(RxBuffer[1]-'0')*10000+(RxBuffer[2]-'0')*1000+(RxBuffer[3]-'0')*100+(RxBuffer[4]-'0')*10+(RxBuffer[5]-'0');
    Angle[2]=(RxBuffer[8]-'0')*10000+(RxBuffer[9]-'0')*1000+(RxBuffer[10]-'0')*100+(RxBuffer[11]-'0')*10+(RxBuffer[12]-'0');
    Angle[1]=(RxBuffer[15]-'0')*10000+(RxBuffer[16]-'0')*1000+(RxBuffer[17]-'0')*100+(RxBuffer[18]-'0')*10+(RxBuffer[19]-'0');
    Angle[3]=(RxBuffer[22]-'0')*10000+(RxBuffer[23]-'0')*1000+(RxBuffer[24]-'0')*100+(RxBuffer[25]-'0')*10+(RxBuffer[26]-'0');
    Angle[4]=(RxBuffer[29]-'0')*10000+(RxBuffer[30]-'0')*1000+(RxBuffer[31]-'0')*100+(RxBuffer[32]-'0')*10+(RxBuffer[33]-'0');
    Angle[5]=(RxBuffer[36]-'0')*10000+(RxBuffer[37]-'0')*1000+(RxBuffer[38]-'0')*100+(RxBuffer[39]-'0')*10+(RxBuffer[40]-'0');

    if(Angle[0]>4500) Angle[0]=4500;
    if(Angle[2]>4500) Angle[2]=4500;
    if(RxBuffer[0]=='0') Angle[0]=9000+Angle[0];
    else Angle[0]=9000-Angle[0];
    if(RxBuffer[7]=='0') Angle[2]=9000+Angle[2];
    else Angle[2]=9000-Angle[2];

    if(Angle[1]>4500) Angle[1]=4500;
    if(Angle[3]>4500) Angle[3]=4500;
    if(RxBuffer[14]=='0') Angle[1]=9000+Angle[1];
    else Angle[1]=9000-Angle[1];
    if(RxBuffer[21]=='0') Angle[3]=9000+Angle[3];
    else Angle[3]=9000-Angle[3];
	
    if(Angle[4]>4500) Angle[4]=4500;
    if(Angle[5]>4500) Angle[5]=4500;
    if(RxBuffer[28]=='0') Angle[4]=9000+Angle[4];
    else Angle[4]=9000-Angle[4];
    if(RxBuffer[35]=='0') Angle[5]=9000+Angle[5];
    else Angle[5]=9000-Angle[5];
    //HAL_UART_Transmit_IT(&huart6,RxBuffer,length);
    HAL_UART_Receive_IT(&huart6,RxBuffer,length);
	
	
	
	
}

/*void anglectrl_servo_2(){
	  if(counter<=1150) HAL_GPIO_WritePin(GPIOB, Port1_Pin, GPIO_PIN_SET);//Angle[0]/9.0+500
	  else HAL_GPIO_WritePin(GPIOB, Port1_Pin, GPIO_PIN_RESET);
}*/



