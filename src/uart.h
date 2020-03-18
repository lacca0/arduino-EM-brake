//
// Created by lacca on 18.03.2020.
//

#pragma once

#include <stdio.h>

extern FILE uart_in, uart_out;

enum uart_error
{
    UART_OK = 0,
    UART_FRAME_ERROR,
    UART_DATA_OVERRUN,
    UART_PARITY_ERROR,
};

void USART_Init(unsigned int ubrr);
void USART_Transmit(char data);
void USART_Transmit_string (char string[]);
void USART_Transmit_string_alt(char string[]);
uint8_t USART_Receive(enum uart_error *error);
int USART_Put_Char(char c, FILE *stream);
int USART_Get_Char(FILE *stream);




