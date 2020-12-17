#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "misc.h"
#include <string.h>
#include "logic.h"

#define SYSCLK 72000000
#define PRESCALER 72
#define PWM_FREQ 50
#define RX_BUF_SIZE 70
#define SECTIONS_SIZE 6

uint16_t SECTION_0_CHANNEL_0 = 1;
uint16_t SECTION_0_CHANNEL_1 = 2;
uint16_t SECTION_1_CHANNEL_0 = 4;
uint16_t SECTION_1_CHANNEL_1 = 8;
uint16_t SECTION_2_CHANNEL_0 = 64;
uint16_t SECTION_2_CHANNEL_1 = 128;
uint16_t SECTION_3_CHANNEL_0 = 16;
uint16_t SECTION_3_CHANNEL_1 = 32;
uint16_t SECTION_4_CHANNEL_0 = 256;
uint16_t SECTION_4_CHANNEL_1 = 512;
uint16_t SECTION_5_CHANNEL_0 = 1024;
uint16_t SECTION_5_CHANNEL_1 = 2048;


uint16_t pwm_array[SECTIONS_SIZE-1];

/*
SECTION_0_PWM = TIM2->CCR3
SECTION_1_PWM = TIM2->CCR4
SECTION_2_PWM = TIM3->CCR1
SECTION_3_PWM = TIM3->CCR2
SECTION_4_PWM = TIM3->CCR3
SECTION_5_PWM = TIM3->CCR4
*/


volatile int isControlMessageReceived = 0;
volatile char RXi;
volatile char RXc;
unsigned char RX_BUF[RX_BUF_SIZE] = {'\0'};
unsigned char control_message[6] = {'\0'};
int JoysticLeftYAxis;
uint16_t zero_message[6];
uint16_t message[6];
int first_msg = 1;


void clear_RXBuffer(void) {
    for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
        RX_BUF[RXi] = '\0';
    RXi = 0;
}

void pwm_init() {
  GPIO_InitTypeDef portA;
  GPIO_InitTypeDef portB;
  TIM_TimeBaseInitTypeDef timer;
  TIM_OCInitTypeDef timerPWM;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_StructInit(&portA);
  portA.GPIO_Mode = GPIO_Mode_AF_PP;
  portA.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
  portA.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &portA);
  
  GPIO_StructInit(&portB);
  portB.GPIO_Mode = GPIO_Mode_AF_PP;
  portB.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  portB.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &portB);

  TIM_TimeBaseStructInit(&timer);
  timer.TIM_Prescaler = PRESCALER;
  timer.TIM_Period = SYSCLK / PRESCALER / PWM_FREQ;
  timer.TIM_ClockDivision = 0;
  timer.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &timer);
  TIM_TimeBaseInit(TIM3, &timer);

  TIM_OCStructInit(&timerPWM);
  timerPWM.TIM_Pulse = 0;
  timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
  timerPWM.TIM_OutputState = TIM_OutputState_Enable;
  timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC3Init(TIM2, &timerPWM);
  TIM_OC4Init(TIM2, &timerPWM);
  TIM_OC1Init(TIM3, &timerPWM);
  TIM_OC2Init(TIM3, &timerPWM);
  TIM_OC3Init(TIM3, &timerPWM);
  TIM_OC4Init(TIM3, &timerPWM);

  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

void usart_init(void) {
    /* Enable USART1 and GPIOA clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
 
    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Configure the GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;
 
    /* Configure USART1 Tx (PB.10) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* Configure USART1 Rx (PB.11) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* Configure the USART1 */
    USART_InitTypeDef USART_InitStructure;
 
    /* USART3 configuration ------------------------------------------------------*/
    /* USART3 configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the middle
        - USART LastBit: The clock pulse of the last data bit is not output to
            the SCLK pin
     */
    USART_InitStructure.USART_BaudRate = 38400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    USART_Init(USART3, &USART_InitStructure);
 
    /* Enable USART1 */
    USART_Cmd(USART3, ENABLE);
 
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
        USART1 receive data register is not empty */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void USART3_IRQHandler(void) {
    if ((USART3->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
      RXc = USART_ReceiveData(USART3);
      
      RX_BUF[RXi] = RXc;
      RXi++;
      
      if (RXi > RX_BUF_SIZE-1) {
        clear_RXBuffer();
      }

      int window_size = 7;
      for (int i=0;i<RX_BUF_SIZE-window_size;i++) {
        
        if ((RX_BUF[i] == 13) && (RX_BUF[i+window_size] == 13)) {
          if (first_msg) {
            first_msg = 0;
            for (int j=0; j<6;++j){
              zero_message[j] = RX_BUF[j + i + 1];
            } 
          } else {
            for (int j=0; j<6;++j){
              message[j] = RX_BUF[j + i + 1];
            }
            isControlMessageReceived = 1;
          }
          clear_RXBuffer();
        }
       
      }
      
    }
}

void USARTSend(const unsigned char *pucBuffer) {
    while (*pucBuffer)
    {
        USART_SendData(USART3, *pucBuffer++);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        {
        }
    }
}

void SetSysClockTo72(void) {
    ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
 
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    if (HSEStartUpStatus == SUCCESS)
    {
      /* Enable Prefetch Buffer */
      //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

      /* Flash 2 wait state */
      //FLASH_SetLatency( FLASH_Latency_2);

      /* HCLK = SYSCLK */
      RCC_HCLKConfig( RCC_SYSCLK_Div1);

      /* PCLK2 = HCLK */
      RCC_PCLK2Config( RCC_HCLK_Div1);

      /* PCLK1 = HCLK/2 */
      RCC_PCLK1Config( RCC_HCLK_Div2);

      /* PLLCLK = 8MHz * 9 = 72 MHz */
      RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

      /* Enable PLL */
      RCC_PLLCmd( ENABLE);

      /* Wait till PLL is ready */
      while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
      {
      }

      /* Select PLL as system clock source */
      RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

      /* Wait till PLL is used as system clock source */
      while (RCC_GetSYSCLKSource() != 0x08)
      {
      }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */
 
      /* Go to infinite loop */
      while (1)
      {
      }
    }
}

void spi_init() {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  SPI_InitTypeDef spi;
  SPI_StructInit(&spi);
  
  spi.SPI_Direction = SPI_Direction_1Line_Tx;
  spi.SPI_Mode = SPI_Mode_Master;
  spi.SPI_DataSize = SPI_DataSize_16b;
  spi.SPI_CPOL = SPI_CPOL_High;
  spi.SPI_CPHA = SPI_CPHA_2Edge;
  spi.SPI_NSS = SPI_NSS_Soft;
  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  spi.SPI_FirstBit = SPI_FirstBit_MSB;
  spi.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &spi);
  
  GPIO_InitTypeDef GPIO_Config;
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
  
  SPI_Cmd(SPI2, ENABLE);
  
  GPIO_ResetBits(GPIOA, GPIO_Pin_9); // enable 30
  GPIO_SetBits(GPIOA, GPIO_Pin_8); //CLR
}

void transfer_16b_SPI2_Master(uint16_t outByte){
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
    GPIO_SetBits(GPIOA, GPIO_Pin_10); // Set A10 to High level ("1")
  
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI2, outByte);
    
    while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));

    //DelayUs(20);
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
    GPIO_ResetBits(GPIOA, GPIO_Pin_10); // Set A10 to Low level ("0")
    //DelayUs(20);
}

void stop_all_sections() {
  transfer_16b_SPI2_Master(0);
  TIM2->CCR3 = 0;
  TIM2->CCR4 = 0;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
}

int main(void)
{
  
  SetSysClockTo72();
  pwm_init();
  usart_init();
  spi_init();
  DelayInit();

  while(1)
  {
    if (isControlMessageReceived) {
      
      uint16_t sections_pwm[6] = {0,0,0,0,0,0};
      uint16_t section_channel = 0;
      parse_section_pwm_and_channel_from_message(zero_message, message, sections_pwm, &section_channel); 
      
      TIM2->CCR3 = sections_pwm[0] * 155;
      TIM2->CCR4 = sections_pwm[1] * 155;
      TIM3->CCR1 = sections_pwm[2] * 155;
      TIM3->CCR2 = sections_pwm[3] * 155;
      TIM3->CCR3 = sections_pwm[4] * 155;
      TIM3->CCR4 = sections_pwm[5] * 155;
      transfer_16b_SPI2_Master(section_channel);
     
      isControlMessageReceived = 0;
      clear_RXBuffer();
    } else {
      //stop_all_sections();
    }

    DelayMs(10);
  }
}