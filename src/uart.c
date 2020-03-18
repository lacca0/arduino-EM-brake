//
// Created by lacca on 18.03.2020.
//

#include <stdio.h>
#include <avr/io.h>
#include "uart.h"

void USART_Init(unsigned int ubrr)
{
    /*Set baud rate */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    /*Enable receiver and transmitter */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    /* Set frame format: 8data, 1stop bit */
    UCSR0C =   (0 << USBS0) | (3 << UCSZ00)
               | (0 << U2X0) | (0 << UMSEL00) | (0 << UMSEL01); // divider 16, asynchronous


}

void USART_Transmit(char data)
{
    /* Wait for empty transmit buffer */
    while (!( UCSR0A & (1 << UDRE0)));
    /* Put data into buffer, sends the data */
    UDR0 = data;
}

uint8_t USART_Receive(enum uart_error *error)
{
    /*Wait for data to be received*/
    while (!(UCSR0A & (1 << RXC0)));

    if (error != NULL) {
        uint8_t ucsr0a = UCSR0A;

        if (ucsr0a & (1 << FE0)) {
            *error = UART_FRAME_ERROR;
        } else if (ucsr0a & (1 << UPE0)) {
            *error = UART_PARITY_ERROR;
        } else if (ucsr0a & (1 << DOR0)) {
            *error = UART_DATA_OVERRUN;
        }
    }
    return UDR0;

}

void USART_Transmit_string(char string[])
{
    int i = 0;
    while (string[i] != 0x00)
    {
        USART_Transmit(string[i]);
        i++;
    }
}

void USART_Transmit_string_alt(char string[])
{
    int i = 0;
    while (string[i] != 0x00) {
        if (string[i] == '\n') {
            USART_Transmit('\r');
        }
        USART_Transmit(string[i]);
        i++;
    }
}

int USART_Put_Char(char c, FILE *stream)
{
    if (c == '\n') {
        USART_Transmit('\r');
    }
    USART_Transmit(c);
    return 0;
}

int USART_Get_Char(FILE *stream)
{
    enum uart_error err;
    uint8_t r = USART_Receive(&err);

    if (err != UART_OK) {
        return _FDEV_ERR;
    }
    return r;
}

FILE uart_in = FDEV_SETUP_STREAM(NULL, USART_Get_Char, _FDEV_SETUP_READ);
FILE uart_out = FDEV_SETUP_STREAM(USART_Put_Char, NULL, _FDEV_SETUP_WRITE);


