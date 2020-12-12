

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"
#include "delay.h"
#include "stm32f10x_it.h"

#define CMD_SET_READ_BIT 0x80
#define REG_WHO_AM_I 0x0F
#define VAL_WHO_AM_I 0xD7
// transfer a byte over SPI2 B12/SS, B13/SCK, B14/MISO, B15/MOSI
void transfer_8b_SPI2_Master(uint8_t outByte){
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI2, outByte);
    
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
    DelayUs(20);
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
    DelayUs(20);

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
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 0;
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
    GPIO_Config.GPIO_Pin = GPIO_Pin_13;//sck  ok
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_15;//mosi ok
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    

    
    GPIO_Config.GPIO_Pin = GPIO_Pin_8;//clr  high
    GPIO_Config.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Config);

    GPIO_Config.GPIO_Pin = GPIO_Pin_9;//enable 30 LOW
    GPIO_Config.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_10;//store (output enable) rck pulse
    GPIO_Config.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_Config);
    
    //uart in/out
    GPIO_Config.GPIO_Pin = GPIO_Pin_11;
    GPIO_Config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_10;
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
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
    TIM_OCConfig.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 0;
    TIM_OC2Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 0;
    TIM_OC3Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 0;
    TIM_OC4Init(TIM3, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 0;
    TIM_OC3Init(TIM2, &TIM_OCConfig);
    
    TIM_OCConfig.TIM_Pulse = 0;
    TIM_OC4Init(TIM2, &TIM_OCConfig);
    


    // timer is on
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    
    //spi on
    SPI_Cmd(SPI2, ENABLE);
}


void UARTInit(void){
    // uart
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    USART_InitTypeDef USART_InitStructure;
    
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

void send_Uart(USART_TypeDef* USARTx, unsigned char c) // отправить байт
{
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE)== RESET){}
	USART_SendData(USARTx, c);
}

unsigned char getch_Uart(USART_TypeDef* USARTx)  //  получить байт
{
	while(USART_GetFlagStatus(USARTx,USART_FLAG_RXNE) == RESET){}
	return USART_ReceiveData(USARTx);
}

int main()
{
  uint8_t gch=0;
  uint8_t spi1=0;
  uint8_t pwm=0;
  
  uint8_t tl1=0;
  uint8_t tl2=0;
  //uint8_t spi2=0;
  
    //PeripheralInit_SPI2_Master();
  InitAll();
  UARTInit();
  DelayInit();
    

  GPIO_ResetBits(GPIOA, GPIO_Pin_9); // enable 30
  GPIO_SetBits(GPIOA, GPIO_Pin_8); //CLR
  //GPIO_ReetBits(GPIOA, GPIO_Pin_10);
  
  //TIM3->CCR1 = 99;
  //TIM3->CCR2 = 99;
  //TIM3->CCR3 = 100;
  //TIM3->CCR4 = 100;
  
  //TIM2->CCR3 = 100;
  //TIM2->CCR4 = 100;
  
    //TIM_OCConfig.TIM_Pulse = 49;
    //TIM_OC4Init(TIM3, &TIM_OCConfig);
    
    while(1)
    {
      
      //TIM_SetCompare3(TIM2,99);
        while(gch==0) {//gch==0
          gch=getch_Uart(USART3);
        }//ожидание сообщения (надо на прерываниях)
        
        //send_Uart(USART3, gch);
        
        //transfer_8b_SPI2_Master();
        //Передача сигнала на шим
        
        tl1=gch >> 4;
        
        if ((tl1)>0) {
        //первый шим
          
        if ((tl1)==1 || (tl1)==2) {
        
          if((tl1)==1) {
            spi1 &= ~(1<<1);//сдвиговый 2 выкл
            spi1 |= 1;     //1 вкл
          }
          else
          {
            spi1 |= (1<<1);//сдвиговый 2 вкл
            spi1 &= ~1;   //1 выкл
          }
        
          transfer_8b_SPI2_Master(spi1);//выбор катушки в сдвиговых
          pwm = ((gch & 15)*6);
          TIM_SetCompare3(TIM2,pwm);
          //TIM2->CCR3 = (gch & 15)*6;   //выбор скважности ШИМ канала 1-2
        }
        
        /*
                //второй шим
        if ((gch >> 4)==3 || (gch >> 4)==4) {
        
          if((gch >> 4)==4) {
            spi1 &= ~(1<<2);//сдвиговый 3 выкл
            spi1 |= (1<<3);     //4 вкл
          }
          else
          {
            spi1 |= (1<<2);//сдвиговый 3 вкл
            spi1 &= ~(1<<3);   //4 выкл
          }
        
          transfer_8b_SPI2_Master(spi1);//выбор катушки в сдвиговых
          TIM_SetCompare4(TIM2,((gch & 15)*6));
          //TIM2->CCR4 = (gch & 15)*6;   //выбор скважности ШИМ канала 3-4
        }

        
              //3 шим
        if ((gch >> 4)==5 || (gch >> 4)==6) {
        
          if((gch >> 4)==6) {
            spi1 &= ~(1<<4);//сдвиговый 5 выкл
            spi1 |= (1<<5);     //6 вкл
          }
          else
          {
            spi1 |= (1<<4);//сдвиговый 5 вкл
            spi1 &= ~(1<<5);   //6 выкл
          }
        
          transfer_8b_SPI2_Master(spi1);//выбор катушки в сдвиговых
          TIM_SetCompare1(TIM3,((gch & 15)*6));
          //TIM3->CCR1 = (gch & 15)*6;   //выбор скважности ШИМ канала 3-4
        }
        
        
           //4 шим
        if ((gch >> 7)==3 || (gch >> 8)==4) {
        
          if((gch >> 4)==8) {
            spi1 &= ~(1<<6);//сдвиговый 7 выкл
            spi1 |= (1<<7);     //8 вкл
          }
          else
          {
            spi1 |= (1<<6);//сдвиговый 7 вкл
            spi1 &= ~(1<<7);   //8 выкл
          }
        
          transfer_8b_SPI2_Master(spi1);//выбор катушки в сдвиговых
          TIM_SetCompare2(TIM3,((gch & 15)*6));
          //TIM3->CCR2 = (gch & 15)*6;   //выбор скважности ШИМ канала 3-4
        }
        
         */
        
      }
      gch=0;

    }
    /*
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
    */
    //return 0;
}
