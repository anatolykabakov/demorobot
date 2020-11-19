#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h" 
#include "stm32f10x_adc.h" 
#include "stm32f10x_it.h" 

#define ARRAYSIZE 8*4
#define ADC1_DR    ((uint32_t)0x4001244C)
volatile uint16_t ADC_values[ARRAYSIZE];
volatile uint32_t status = 0;

#include "delay.h"
#include "lcd16x2.h"
#include "misc.h" 

char value[3];

// Custom char data (battery symbol)
//uint8_t custom_char[] = { 0x0E, 0x1B, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F };
uint8_t custom_char[] =       { 0x00, 0x0A, 0x1F, 0x1F, 0x0E, 0x04, 0x00, 0x00 };


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

void ADCInit(void){
    //--Enable ADC1 and GPIOA--
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
    //==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs==
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure, if not it can cause issues...
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6| GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    //ADC1 configuration

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    //We will convert multiple channels
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    //select continuous conversion mode
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//!
    //select no external triggering
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    //right 12-bit data alignment in ADC data register
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //8 channels conversion
    ADC_InitStructure.ADC_NbrOfChannel = 4;
    //load structure values to control and status registers
    ADC_Init(ADC1, &ADC_InitStructure);
    //wake up temperature sensor
    //ADC_TempSensorVrefintCmd(ENABLE);
    //configure each channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_41Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_41Cycles5);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_41Cycles5);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_41Cycles5);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 7, ADC_SampleTime_41Cycles5);
    //ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_41Cycles5);
    //Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    //enable DMA for ADC
    ADC_DMACmd(ADC1, ENABLE);
    //Enable ADC1 reset calibration register
    ADC_ResetCalibration(ADC1);
    //Check the end of ADC1 reset calibration register
    while(ADC_GetResetCalibrationStatus(ADC1));
    //Start ADC1 calibration
    ADC_StartCalibration(ADC1);
    //Check the end of ADC1 calibration
    while(ADC_GetCalibrationStatus(ADC1));
}

void DMAInit(void){
    //enable DMA1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //create DMA structure
    DMA_InitTypeDef  DMA_InitStructure;
    //reset DMA1 channe1 to default values;
    DMA_DeInit(DMA1_Channel1);
    //channel will be used for memory to memory transfer
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    //setting normal mode (non circular)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    //medium priority
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //source and destination data size word=32bit
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    //automatic memory destination increment enable.
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //source address increment disable
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //Location assigned to peripheral register will be source
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    //chunk of data to be transfered
    DMA_InitStructure.DMA_BufferSize = ARRAYSIZE;
    //source and destination start addresses
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_values;
    //send values to DMA registers
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    // Enable DMA1 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1
    
    NVIC_InitTypeDef NVIC_InitStructure;
    //Enable DMA1 channel IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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

void ValueStr(int i){
i=i>>4;
int sch=0;
 
while (i>99){
sch=sch+1;
i=i-100;
}

value[0]=sch+'0';
sch=0;

while(i>9){
sch=sch+1;
i=i-10;
}

value[1]=sch+'0';
value[2]=i+'0';

}

int Norm(int i){
i=i>>4;

return i;
}

int main(void)
{
  uint8_t adc=0;
  uint8_t spi=0;
    
  DelayInit();
 // DelayMs(6000);
  
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );
    //GPIO_PinRemapConfig (GPIO_Remap_SWJ_JTAGDisable, ENABLE );
    //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    
    GPIO_InitTypeDef GPIO_Config;
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); 
    //GPIO_InitTypeDef GPIO_Config;

    
    ADCInit();
    DMAInit();
    UARTInit();
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_11;
    GPIO_Config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_10;
    GPIO_Config.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_15;
    GPIO_Config.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_15;
    GPIO_Config.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_Config);
    
    GPIO_Config.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Config.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Config.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_Config);
    

    
    // Delay initialization
    //DelayInit();
    // LCD initialization
    lcd16x2_init(LCD16X2_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
    // Create custom char
    //lcd16x2_create_custom_char(0, custom_char);

    //Enable DMA1 Channel transfer
    DMA_Cmd(DMA1_Channel1, ENABLE);
    //Start ADC1 Software Conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    //wait for DMA to complete
    while (!status){};
    //DelayMs(100);
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);

    //send_Uart(USART3, 2);
    
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_3);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    
    lcd16x2_gotoxy(0, 0);
    
    /*
    lcd16x2_gotoxy(0, 1);
    
    lcd16x2_puts("START");
    DelayMs(800);
    lcd16x2_clrscr();
    
    lcd16x2_puts("BP4=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)+'0');
    DelayMs(800);
    
    lcd16x2_puts(" BP5=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)+'0');
    DelayMs(800);
    
    lcd16x2_gotoxy(0, 2);
    
    lcd16x2_puts("BP6=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)+'0');
    DelayMs(800);
    
    lcd16x2_puts(" BP7=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)+'0');
    
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_3);
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    
    DelayMs(3000);
    //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
    
    lcd16x2_clrscr();
    
    lcd16x2_puts("BP4=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)+'0');
    DelayMs(800);
    
    lcd16x2_puts(" BP5=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)+'0');
    DelayMs(800);
    
    lcd16x2_gotoxy(0, 2);
    
    lcd16x2_puts("BP6=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)+'0');
    DelayMs(800);
    
    lcd16x2_puts(" BP7=");
    lcd16x2_putc(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)+'0');
    DelayMs(3000);
    
    lcd16x2_clrscr();
    
    
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_3);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    */
    
    while (1)
    {
        // Display custom char
        //lcd16x2_put_custom_char(0, 0, 0);
        //lcd16x2_puts("Рез=");
        //lcd16x2_write_data(getch_Uart(USART3));ADC_values[index])

        adc = ADC_values[3]>>4;
        if ((adc)>127) {
          spi = ((adc-127)>>4)*2;
          spi |= (1<<4);
          spi &= ~(2<<4);
          send_Uart(USART3, spi);
        } else {
          spi = ((127-adc)>>4)*2;
          spi |= (2<<4);
          spi &= ~(1<<4);
          send_Uart(USART3, spi);
        }
        
        lcd16x2_puts("PB0=");
        ValueStr(adc);
        lcd16x2_putc(value[0]);
        lcd16x2_putc(value[1]);
        lcd16x2_putc(value[2]);
        
        adc = ADC_values[1]>>4;
        /*
        DelayMs(10);
        
        adc = ADC_values[1]>>4;
        if ((adc)>127) {
          send_Uart(USART3, ((3<<4)+((adc-127)>>4)*2));
        } else {
          send_Uart(USART3, ((4<<4)+((127-adc)>>4)*2));
        }
        */
        lcd16x2_puts(" PA6=");
        ValueStr(adc);
        lcd16x2_putc(value[0]);
        lcd16x2_putc(value[1]);
        lcd16x2_putc(value[2]);
        
        adc = ADC_values[2]>>4;
        
        lcd16x2_gotoxy(0, 2);
        /*
        DelayMs(10);
        
        
        adc = ADC_values[2]>>4;
        if ((adc)>127) {
          send_Uart(USART3, ((5<<4)+((adc-127)>>4)*2));
        } else {
          send_Uart(USART3, ((6<<4)+((127-adc)>>4)*2));
        }
        */
        
        lcd16x2_puts("PA7=");
        ValueStr(adc);
        lcd16x2_putc(value[0]);
        lcd16x2_putc(value[1]);
        lcd16x2_putc(value[2]);
        
        adc = ADC_values[4]>>4;
        /*
        DelayMs(10);
        
        adc = ADC_values[4]>>4;
        if ((adc)>127) {
          send_Uart(USART3, ((7<<4)+((adc-127)>>4)*2));
        } else {
          send_Uart(USART3, ((8<<4)+((127-adc)>>4)*2));
        }
        */
        lcd16x2_puts(" PA5=");
        ValueStr(adc);
        lcd16x2_putc(value[0]);
        lcd16x2_putc(value[1]);
        lcd16x2_putc(value[2]);
        
        DelayMs(10);
    
        lcd16x2_clrscr();
    }
}