//
// Created by LEGION on 2021/10/4.
//
#include "Device.h"
#include "UserTask.h"
#include "RemoteControl.h"
#include "Motor.h"
#include "IMU.h"


//uint8_t RxBuffer[length]={'0','0','0','0','0','0','0','0','0','0','0','0','0'};
uint8_t RxBuffer[length]={0};
uint8_t RxBuffer2[11]={0};
int32_t Newdata[6]={0};
int32_t Angle[6]={9000,9000,9000,9000,9000,9000};
int32_t sonar=0;
int counter=0;


void aRGB_led_show(uint32_t aRGB){
    static uint8_t alpha;
    static uint16_t red,green,blue;
    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);

}

/**
 * @brief 跑马灯函数，需周期性调用
 * @param period 跑马灯循环周期
 */
void aRGB_led_change(uint32_t period){
    static uint32_t aRGB_value = 0xffff0000;
    static uint32_t lastTick = 0;
    static float lastPhase = 0;
    uint32_t red,green,blue;
    red = ((aRGB_value & 0x00FF0000) >> 16) ;
    green = ((aRGB_value & 0x0000FF00) >> 8) ;
    blue = ((aRGB_value & 0x000000FF) >> 0) ;

    float interval = HAL_GetTick()- lastTick;
    if(lastTick == 0){//处理首次进入
        lastTick = HAL_GetTick();
        return;
    }
    interval = interval * 765/(float)period;
    //if(interval == 0)interval = 1;
    uint32_t phase =  (uint32_t)(interval+ lastPhase)%765;
    uint32_t group = phase/255;
    uint32_t sub_phase = phase%255;

    switch(group){
        case 0:
            red = 255 - sub_phase;
            green = sub_phase;
            blue = 0;
            break;
        case 1:
            red = 0;
            green = 255 - sub_phase;
            blue = sub_phase;
            break;
        case 2:
            red = sub_phase;
            green = 0;
            blue = 255 - sub_phase;
            break;
        default:
            red = 255;
            green = 0;
            blue = 0;
            break;
    }
    aRGB_value = 0xff000000;
    aRGB_value |= red << 16;
    aRGB_value |= green << 8;
    aRGB_value |= blue;

    aRGB_led_show(aRGB_value);

    lastTick = HAL_GetTick();
    lastPhase = lastPhase + interval;
}

uint8_t buzzerWorkingFlag;


float bsp_BuzzerOn(float _freq,float _targetVolPct){

#define BUZZER_CLOCK_FREQUENCY 84000000
#define BUZZER_CLOCK htim4
#define BUZZER_CLOCK_CHANNEL TIM_CHANNEL_3
#define REFERENCE_MAX_VOL_CCR 5000

    uint16_t arr,cpr;
    arr = (uint16_t)((BUZZER_CLOCK_FREQUENCY/ (float)(BUZZER_CLOCK.Instance->PSC+1))/_freq);
    //cpr = (uint16_t)(duty*(float)arr);
    cpr = (uint16_t)(_targetVolPct*REFERENCE_MAX_VOL_CCR*0.5);
    if(cpr > arr/2){
        cpr = arr/2;
    }


    if(!buzzerWorkingFlag){
        buzzerWorkingFlag = 1;
        HAL_TIM_Base_Start(&BUZZER_CLOCK);
        HAL_TIM_PWM_Start(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL);
    }
    BUZZER_CLOCK.Instance->ARR = arr;
            __HAL_TIM_SetCompare(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL,cpr);

    return (float)cpr/(float)arr*2;
}

void bsp_BuzzerOff(){
    if(buzzerWorkingFlag){
        buzzerWorkingFlag = 0;
        HAL_TIM_PWM_Stop(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL);
    }
}

volatile float vccMoni = 0;
volatile float vccBat = 0;

void bsp_ADC_vccMoni(){
    static uint8_t calibrationCplt = 0;
    static uint32_t calibrationCnt = 0;
    static uint32_t vrefSum = 0,vrefValue = 0;
    if(calibrationCplt == 0){
        if (calibrationCnt == 0){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else if(calibrationCnt < 200){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else{
            calibrationCplt = 1;
            vrefValue = vrefSum/200.0f;
            HAL_ADC_Stop(&hadc1);

            HAL_ADC_Start(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 2);
            vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

        }

    }else{
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 2);
        vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

    }
    vccBat = vccMoni/22.0f*222.0f;
}

/**
 * @brief 利用串口重定向函数，可选usb虚拟串口或硬件串口，使用需安装相应驱动
 * @param fmt
 * @param ...
 */
void usart_printf(const char *fmt,...){

    static uint8_t tx_buf[256] = {0};//TODO 爆栈？
    static va_list ap;
    static uint16_t len;

    va_start(ap,fmt);

    len = vsprintf((char*)tx_buf,fmt,ap);

    va_end(ap);

    //HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
    CDC_Transmit_FS(tx_buf,len);

}

#define FLASH_SECTOR_9_ADDRESS 0x080A0000
flash_data_t flashData;
void bsp_flash_write(flash_data_t *_flashData){
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = FLASH_SECTOR_9;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&eraseInit,nullptr);
    uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;

    for (uint32_t *ptr = (uint32_t*)_flashData;ptr < (uint32_t*)(sizeof(flash_data_t)/4 + _flashData);ptr++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,flash_ptr,*ptr);
        flash_ptr += 4;
    }
    HAL_FLASH_Lock();
}
void bsp_flash_read(flash_data_t *_flashData){
    uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;
    memcpy(_flashData,(uint32_t*)FLASH_SECTOR_9_ADDRESS,sizeof(flash_data_t)/4);
}



uint32_t init_Flag = 0;
uint32_t period = 5000;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(init_Flag == 0)return;
    if(htim == &htim10){//1ms
        //HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
        aRGB_led_change(period);

        bsp_ADC_vccMoni();

        static uint32_t cnt = 0;
        cnt++;

        //anglectrl_motor();//控制云台电机用
        //Motor::MotorsHandle();//控制云台电机用
        //CAN::CANPackageSend();//控制云台电机用
        //anglectrl_servo();//控制舵机用
        //speedctrl_propeller();//控制推进器用



        /*CtrlHandle ();
        ChassisHandle();
        ARMHandle();
        Motor::PackageSend();*/

        //UserHandle();
        if(cnt>20){
            if(vccBat<10)HAL_IWDG_Refresh(&hiwdg);
            cnt =0;
        }
    }
    /*if (htim == &htim6){
					
        Motor::RS485PackageSend();

    }*/
    if (htim == &htim7){
        IMU::imu.Handle();
    }
}

volatile uint8_t key_raw_state = 1;//
uint32_t key_last_stamp;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t datasize)//HAL_UART_RxCpltCallback
{//串口接收中断，串口收到数据时执行
	  //HAL_UART_Receive_IT(&huart6,RxBuffer,length);
	  
    //if(huart == &huart1){

    //}
    if(huart->Instance==USART6) {
        //HAL_UART_Transmit_IT(&huart6,RxBuffer,datasize);
        Newdatahandle(RxBuffer,Newdata);
        //SerialRx_2();//接收2组数据
        //SerialRx_4();//接收4组数据
        //SerialRx_6();//接收6组数据
        HAL_UARTEx_ReceiveToIdle_IT(&huart6,RxBuffer,length);

    }
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(init_Flag == 0)return;
    if (GPIO_Pin == GPIO_PIN_0){
        key_raw_state = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
        uint32_t time_interval = HAL_GetTick() - key_last_stamp;
        key_last_stamp = HAL_GetTick();
        if (time_interval >= 50){
            if (key_raw_state == 1){
                period = 5000;
            }else{
                period = 500;
            }
        }
    }
    IMU::imu.ITHandle(GPIO_Pin);
}

void Newdatahandle(uint8_t *rx_buffer, int32_t *pwm_value){
    // 示例：假设 PWM 命令格式是 "PWM:1000,2000,1500,1800,1600,1400\n"
    if (strncmp((char*)rx_buffer, "PWM:", 4) == 0) {
        char *pwm_values_str = (char*)rx_buffer + 4;
        char *token = strtok(pwm_values_str, ",");
        int i = 0;
        while (token != NULL && i < 6) {
            pwm_value[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM5_Init();
    MX_TIM1_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM10_Init();
    MX_TIM8_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
   // MX_IWDG_Init();
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);


    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);
    //TODO adc校准？
    RemoteControl::init();
    bsp_flash_read(&flashData);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_PWM_Start_IT(&htim10, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    HAL_UARTEx_ReceiveToIdle_IT(&huart6,RxBuffer,length);
    //Motor::Init();
    IMU::imu.Init();
    //ChassisStart();
    //UserInit();
    init_Flag = 1;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}