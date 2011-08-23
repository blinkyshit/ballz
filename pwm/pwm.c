#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "debug.h"

// Bit manipulation macros
#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A
#define MAX(a,b) ((a) > (b) ? (a) : (b))

#define PWM_COUNT 24
volatile uint8_t pwm_index = 0;
volatile uint8_t pwm[PWM_COUNT];
uint8_t led_last_state[PWM_COUNT];

void set_led(uint8_t led, uint8_t state);

ISR (TIMER2_OVF_vect)
{
    uint8_t i;

    // See which LEDs need to be turned on
    for(i = 0; i < PWM_COUNT; i++)
    {
        if (pwm_index >= pwm[i])
            set_led(i, 1); 
    }

    pwm_index++;
    // The timer roller over, turn off all LEDs
    if (!pwm_index)
    {
        for(i = 0; i < PWM_COUNT; i++)
           set_led(i, 0); 
    }
}

void timer_setup(void)
{
    //TCCR2B  = 0; // |= _BV(CS22); // | _BV(CS21) | _BV(CS20);
    TCCR2B  = _BV(CS20);
    TCNT2 = 0;
    TIMSK2 |= _BV(TOIE2);
}

void set_led(uint8_t led, uint8_t state)
{
    if (led_last_state[led] == state)
        return;

    led_last_state[led] = state;

    // led 0
    if (led == 0)
    {
        if (state)
            cbi(PORTC, 7);
        else
            sbi(PORTC, 7);
    }
    else
    if (led == 1)
    {
        if (state)
            cbi(PORTA, 7);
        else
            sbi(PORTA, 7);
    }
    else
    if (led == 2)
    {
        if (state)
            cbi(PORTA, 6);
        else
            sbi(PORTA, 6);
    }
    // led 1
    else
    if (led == 3)
    {
        if (state)
            cbi(PORTC, 4);
        else
            sbi(PORTC, 4);
    }
    else
    if (led == 4)
    {
        if (state)
            cbi(PORTC, 5);
        else
            sbi(PORTC, 5);
    }
    else
    if (led == 5)
    {
        if (state)
            cbi(PORTC, 6);
        else
            sbi(PORTC, 6);
    }
    else
    // led 2
    if (led == 6)
    {
        if (state)
            cbi(PORTC, 1);
        else
            sbi(PORTC, 1);
    }
    else
    if (led == 7)
    {
        if (state)
            cbi(PORTC, 2);
        else
            sbi(PORTC, 2);
    }
    else
    if (led == 8)
    {
        if (state)
            cbi(PORTC, 3);
        else
            sbi(PORTC, 3);
    }
    else
    // led 3
    if (led == 9)
    {
        if (state)
            cbi(PORTD, 6);
        else
            sbi(PORTD, 6);
    }
    else
    if (led == 10)
    {
        if (state)
            cbi(PORTD, 7);
        else
            sbi(PORTD, 7);
    }
    else
    if (led == 11)
    {
        if (state)
            cbi(PORTC, 0);
        else
            sbi(PORTC, 0);
    }
    else
    // led 4
    if (led == 12)
    {
        if (state)
            cbi(PORTD, 3);
        else
            sbi(PORTD, 3);
    }
    else
    if (led == 13)
    {
        if (state)
            cbi(PORTD, 4);
        else
            sbi(PORTD, 4);
    }
    else
    if (led == 14)
    {
        if (state)
            cbi(PORTD, 5);
        else
            sbi(PORTD, 5);
    }
    // led 5
    else
    if (led == 15)
    {
        if (state)
            cbi(PORTB, 6);
        else
            sbi(PORTB, 6);
    }
    else
    if (led == 16)
    {
        if (state)
            cbi(PORTB, 7);
        else
            sbi(PORTB, 7);
    }
    else
    if (led == 17)
    {
        if (state)
            cbi(PORTD, 2);
        else
            sbi(PORTD, 2);
    }
    else
    // led 6
    if (led == 18)
    {
        if (state)
            cbi(PORTB, 3);
        else
            sbi(PORTB, 3);
    }
    else
    if (led == 19)
    {
        if (state)
            cbi(PORTB, 4);
        else
            sbi(PORTB, 4);
    }
    else
    if (led == 20)
    {
        if (state)
            cbi(PORTB, 5);
        else
            sbi(PORTB, 5);
    }
    else
    // led 7
    if (led == 21)
    {
        if (state)
            cbi(PORTB, 0);
        else
            sbi(PORTB, 0);
    }
    else
    if (led == 22)
    {
        if (state)
            cbi(PORTB, 1);
        else
            sbi(PORTB, 1);
    }
    else
    if (led == 23)
    {
        if (state)
            cbi(PORTB, 2);
        else
            sbi(PORTB, 2);
    }
}

void set_color(uint8_t tled, uint8_t red, uint8_t green, uint8_t blue)
{
    tled *= 3;

    cli();
    pwm[tled] = 255 - red;
    pwm[tled+1] = 255 - green;
    pwm[tled+2] = 255 - blue;
    sei();
}

void flash_led(void)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        cbi(PORTB, 3);
        _delay_ms(100); 
        sbi(PORTB, 3);
        _delay_ms(100); 
    }
}

int main(void)
{
    uint8_t i, j;

    serial_init();
    //timer_setup();

    DDRA |= (1 << PA6) | (1 << PA7);
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD |= (1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);

    sei();

    for(j = 0; j < 24; j++)
        set_color(j, 0, 0, 0);

    while(1)
    {
        for(j = 0; j < 24; j++)
            set_led(j, 1);
        _delay_ms(100);
        for(j = 0; j < 24; j++)
            set_led(j, 0);
        _delay_ms(100);
    }
    while(0)
    {
        for(j = 0; j < 24; j++)
            set_color(j, 0, 0, 0);
        _delay_ms(100);
        for(j = 0; j < 24; j++)
            set_color(j, 255, 255, 255);
        _delay_ms(100);
    }

    while(0)
    {
        set_color(0, 0x99, 0, 0x99);
        _delay_ms(100);
        set_color(0, 0xFF, 0xCC, 0);
        _delay_ms(100);
    }

    while(1)
    {
        for(i = 0; i < 255; i++)
        {
            for(j = 0; j < 24; j += 3)
                set_color(j, i, 0, 0);
            _delay_ms(10);
        }
        for(i = 0; i < 255; i++)
        {
            for(j = 0; j < 24; j += 3)
                set_color(j + 1, 255, i, 0);
            _delay_ms(10);
        }
        for(i = 0; i < 255; i++)
        {
            for(j = 0; j < 24; j += 3)
                set_color(j + 2, 255, 255, i);
            _delay_ms(10);
        }
    }


    while(0)
    {
        cbi(PORTD, 5);
        _delay_ms(100); 
        sbi(PORTD, 5);
        _delay_ms(100); 
        cbi(PORTD, 6);
        _delay_ms(100); 
        sbi(PORTD, 6);
        _delay_ms(100); 
        cbi(PORTD, 7);
        _delay_ms(100); 
        sbi(PORTD, 7);
        _delay_ms(100); 
    }

	return 0;
}
