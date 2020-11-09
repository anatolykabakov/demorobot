

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"

#define CMD_SET_READ_BIT 0x80
#define REG_WHO_AM_I 0x0F
#define VAL_WHO_AM_I 0xD7
// transfer a byte over SPI2 B12/SS, B13/SCK, B14/MISO, B15/MOSI
void transfer_8b_SPI2_Master(uint8_t outByte){
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI2, outByte);
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
}


void InitAll()
{
    //SPI_2.begin()
    //clk on timers 3-5 and port A //need 3 and 2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    
    //spi
    SPI_InitTypeDef spi;
    SPI_StructInit(&spi);

    spi.SPI_Direction = SPI_Direction_1Line_Tx;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &spi);
    
    // port config pwm
    GPIO_InitTypeDef GPIO_Config;
    GPIO_Config.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_Config);

    // port config pwm
    GPIO_Config.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_Config);

    //port config spi
    GPIO_Config.GPIO_Pin = GPIO_Pin_13;
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_15;
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Config.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    // port config output
    GPIO_Config.GPIO_Pin = GPIO_Pin_10;
    GPIO_Config.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_Config);
    
    // Timer config
    TIM_TimeBaseInitTypeDef TIM_BaseConfig;
    // Timer output
    TIM_OCInitTypeDef TIM_OCConfig;

    // stsrt timer at 10000 kHz clk
    TIM_BaseConfig.TIM_Prescaler = (uint16_t) (SystemCoreClock / 100000) - 1;
    // period 10 => 10000/10 = 1 kHz
    TIM_BaseConfig.TIM_Period = 99;
    TIM_BaseConfig.TIM_ClockDivision = 0;
    // count from zero to TIM_Period
    TIM_BaseConfig.TIM_CounterMode = TIM_CounterMode_Up;

    // initilayze timer 3 (outputs port A)
    TIM_TimeBaseInit(TIM3, &TIM_BaseConfig);
    TIM_TimeBaseInit(TIM2, &TIM_BaseConfig);

    // config timer output, mode - PWM1
    TIM_OCConfig.TIM_OCMode = TIM_OCMode_PWM1;
    // output enable
    TIM_OCConfig.TIM_OutputState = TIM_OutputState_Enable;
    // pulse length => 5/10 = 50%
    TIM_OCConfig.TIM_Pulse = 4;
    // PWM POLARITY - POSITIVE (+3.3V)
    TIM_OCConfig.TIM_OCPolarity = TIM_OCPolarity_High;

    // Initilize first out of timer 3 (PA6)
    TIM_OCConfig.TIM_Pulse = 89;
    TIM_OC1Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 74;
    TIM_OC2Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 59;
    TIM_OC3Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 44;
    TIM_OC4Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 29;
    TIM_OC3Init(TIM2, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 14;
    TIM_OC4Init(TIM2, &TIM_OCConfig);
    


    // timer is on
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    
    //spi on
    SPI_Cmd(SPI2, ENABLE);
}


int main()
{
    //PeripheralInit_SPI2_Master();
  InitAll();
 
    while(1)
    {
transfer_8b_SPI2_Master(170); // 10101010
      //SPI_I2S_SendData(SPI2, sendData);
    }
    return 0;
}
