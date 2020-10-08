#include "uart.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

// ==================== function implementations ====================
void usart1_init(unsigned int baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    USART_ClockInitTypeDef usartclock;
    NVIC_InitTypeDef nvic;
    // Enable all GPIO and USART clocks needed for USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // Create gpio struct and fill it with defaults
    GPIO_StructInit(&gpio);

    // Set PA9 to alternate function push pull (Tx)
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &gpio);
    // Set PA10 to input floating (Rx)
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &gpio);

    // Create usart struct and init USART1 to 115 200 baud
    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No ;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    // Init USART1 clock
    USART_ClockStructInit(&usartclock);
    USART_ClockInit(USART1, &usartclock);   
    // Init NVIC for USART1 RXNE
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&nvic);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // Enable USART1
    USART_Cmd(USART1, ENABLE);
}
void usart1_send_char(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, c);
}
void usart1_send_string(char* str)
{
    // Sends a string, character for character, over USART1
    while (*str)
        usart1_send_char(*str++);
}
// ==================== interrupt handler ====================
void USART1_IRQHandler()
{
    // echo everything
    USART_SendData(USART1, USART_ReceiveData(USART1));
}
