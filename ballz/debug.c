// 
// Copyright (c) Party Robotics LLC 2010
// Written by Robert Kaye <rob@partyrobotics.com>
//
#include <avr/io.h>
#include <stdarg.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define BAUD 9600 
#define BAUD_CONST (F_CPU / 16 / BAUD - 1)

#define _UBRRH UBRRH
#define _UBRRL UBRRL
#define _UCSRB UCSRB
#define _UCSRC UCSRC
#define _TXEN  TXEN
#define _RXEN  RXEN
#define _RXC   RXC
#define _USBS  USBS
#define _UCSZ1 UCSZ1
#define _UCSZ0 UCSZ0
#define _UCSRA UCSRA
#define _UDRE  UDRE
#define _UDR   UDR 

// TODO: This section needs to be customized for each AVR chip.
void serial_init(void)
{
    // UART 0
    /*Set baud rate */ 
    _UBRRH = (unsigned char)(BAUD_CONST>>8); 
    _UBRRL = (unsigned char)BAUD_CONST; 

    /* Enable transmitter */ 
    _UCSRB = (1<<_TXEN)|(1<<_RXEN); 
    /* Set frame format: 8data, 1stop bit */ 
    _UCSRC = (0<<_USBS)|(1<<_UCSZ0)|(1<<_UCSZ1); 
}

void serial_tx(unsigned char ch)
{
    while ( !( _UCSRA & (1<<_UDRE)) )
        ;

    _UDR = ch;
}

unsigned char serial_rx(void)
{
    while ( !(_UCSRA & (1<<_RXC))) 
        ;

    return _UDR;
}

#define MAX 80 
void dprintf(const char *fmt, ...)
{
    va_list va;
    va_start (va, fmt);

    char buffer[MAX];
    char *ptr = buffer;
    vsnprintf(buffer, MAX, fmt, va);
    va_end (va);

    for(ptr = buffer; *ptr; ptr++)
		{
		   if (*ptr == '\n') serial_tx('\r');
       serial_tx(*ptr);
    }
}
