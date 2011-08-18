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

#define PWM_COUNT 3
volatile uint8_t pwm_index = 0;
volatile uint8_t pwm[PWM_COUNT];

void set_led(uint8_t led, uint8_t state);

ISR (TIMER2_OVF_vect)
{
    uint8_t i;

    // See which LEDs need to be turned on
    for(i = 0; i < PWM_COUNT; i++)
    {
        if (pwm_index == pwm[i])
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
    if (led == 0)
    {
        if (state)
            cbi(PORTD, 5);
        else
            sbi(PORTD, 5);
    }
    else
    if (led == 1)
    {
        if (state)
            cbi(PORTD, 6);
        else
            sbi(PORTD, 6);
    }
    else
    if (led == 2)
    {
        if (state)
            cbi(PORTD, 7);
        else
            sbi(PORTD, 7);
    }
}

void set_color(uint8_t tled, uint8_t red, uint8_t green, uint8_t blue)
{
    tled *= 3;

    pwm[tled] = 255 - red;
    pwm[tled+1] = 255 - green;
    pwm[tled+2] = 255 - blue;
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
    uint8_t i;

    serial_init();
    timer_setup();

    DDRB |= (1 << PB3);
    DDRD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
    sbi(PORTD, 5);
    sbi(PORTD, 6);
    sbi(PORTD, 7);

    sei();

    while(1)
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
            set_color(0, i, 0, 0);
            _delay_ms(10);
        }
        for(i = 0; i < 255; i++)
        {
            set_color(0, 255, i, 0);
            _delay_ms(10);
        }
        for(i = 0; i < 255; i++)
        {
            set_color(0, 255, 255, i);
            _delay_ms(10);
        }
    }

    while(0)
    {
        set_led(0, 1);
        _delay_ms(100);
        set_led(0, 0);
        _delay_ms(100);
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
