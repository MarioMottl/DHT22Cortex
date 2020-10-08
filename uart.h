#ifndef UART_H
#define UART_H

//#include "stm32f10x_usart.h"

void usart1_init(unsigned int baudrate);
void usart1_send_char(char c);
void usart1_send_string(char* s);

#endif // UART_H
