#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "debug.h"

#define USE_TIMER

// Bit manipulation macros
#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A
#define MAX(a,b) ((a) > (b) ? (a) : (b))
//#define bit(a,b) ((&(a)[b>>3]) & 1 << (b & 0x7))
#define bit(a,b) ((a) & 1 << (b & 0x7))

#define PWM_COUNT 24
volatile uint8_t pwm_index = 0;
volatile uint8_t pwm[PWM_COUNT];
volatile uint8_t led_last_state[PWM_COUNT];

void set_led(uint8_t led, uint8_t state);

void _manual_isr(void);

ISR (TIMER2_OVF_vect)
{
    _manual_isr();
}

// [0] = red, [1] = green, [2] = blue, all full power. [4] = red 254/255 power, etc. Each bit is one LED.
volatile uint8_t led_power_map[768];

void _manual_isr(void) {
    static uint16_t counter = 0;
    uint8_t i;

    if (led_power_map[counter])
        for (i=0; i<8; i++)
            if (bit(led_power_map[counter], i))
                set_led(i*3 + 0, 1);
    counter++;
    if (led_power_map[counter])
        for (i=0; i<8; i++)
            if (bit(led_power_map[counter], i))
                set_led(i*3 + 1, 1);
    counter++;
    if (led_power_map[counter])
        for (i=0; i<8; i++)
            if (bit(led_power_map[counter], i))
                set_led(i*3 + 2, 1);
    counter++;

    // The timer rolled over, turn off all LEDs
    if (counter == 765)
    {
        counter = 0;
#if 1
        for(i = 0; i < 8; i++)
           set_led(i*3, bit(led_power_map[0], i));
        for(i = 0; i < 8; i++)
           set_led(i*3 + 1, bit(led_power_map[1], i));
        for(i = 0; i < 8; i++)
           set_led(i*3 + 2, bit(led_power_map[2], i));
#else
        for (i=0; i<PWM_COUNT; i++)
            set_led(i, 0);
#endif
    }
}   

void manual_isr(void) {
    int i;
    for (i=0; i<256; i++)
        _manual_isr();
}

void pwm_timer_setup(void)
{
    //TCCR2B  = 0; // |= _BV(CS22); // | _BV(CS21) | _BV(CS20);
    TCCR2B  = _BV(CS20);
    TCNT2 = 0;
    TIMSK2 |= _BV(TOIE2);
}

// PORTA = 1, PORTB = 2, PORTC = 3, PORTD = 4
static const uint8_t port[PWM_COUNT] =
    {3, 1, 1, 3, 3, 3, 3, 3, 3, 4, 4, 3, 4, 4, 4, 2, 2, 4, 2, 2, 2, 2, 2, 2};
static const uint8_t line[PWM_COUNT] =
    {7, 7, 6, 4, 5, 6, 1, 2, 3, 6, 7, 0, 3, 4, 5, 6, 7, 2, 3, 4, 5, 0, 1, 2};

void set_led(uint8_t led, uint8_t state)
{
    if (led >= PWM_COUNT)
        return;

    // ensure state is either 0 or 1
    state = !!state;
    
    if (led_last_state[led] == state)
        return;

    led_last_state[led] = state;

    switch(port[led] + 4*state)
    {
        case 1: sbi(PORTA, line[led]); break;
        case 2: sbi(PORTB, line[led]); break;
        case 3: sbi(PORTC, line[led]); break;
        case 4: sbi(PORTD, line[led]); break;
        case 5: cbi(PORTA, line[led]); break;
        case 6: cbi(PORTB, line[led]); break;
        case 7: cbi(PORTC, line[led]); break;
        case 8: cbi(PORTD, line[led]); break;
    }
}

void set_color(uint8_t tled, uint8_t red, uint8_t green, uint8_t blue)
{
    static uint8_t last_duty[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    if (tled >= 8)
        return;

    uint8_t led_mask = 1 << tled;

    red = ~red;
    green = ~green;
    blue = ~blue;

    cli();
    led_power_map[last_duty[tled*3] * 3] &= ~led_mask;
    led_power_map[last_duty[tled*3 + 1] * 3 + 1] &= ~led_mask;
    led_power_map[last_duty[tled*3 + 2] * 3 + 2] &= ~led_mask;

    led_power_map[red * 3   + 0] |= led_mask;
    led_power_map[green * 3 + 1] |= led_mask;
    led_power_map[blue * 3  + 2] |= led_mask;
    sei();
    
    last_duty[tled*3 + 0] = red;
    last_duty[tled*3 + 1] = green;
    last_duty[tled*3 + 2] = blue;
}

#if 0
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

void delay(uint8_t x) {
#ifdef USE_TIMER
    _delay_ms(x);
#else
    while (x--)
        manual_isr();
#endif
}

int main(void)
{
    uint16_t i, j, loop;

    for (i=0; i<PWM_COUNT; i++)
        led_last_state[i] = 2;
    for (i=0; i<768; i++)
        led_power_map[i] = 0;

    serial_init();

#ifdef USE_TIMER
    timer_setup();
#endif

    DDRA |= (1 << PA6) | (1 << PA7);
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD |= (1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);

    sei();

    for(j = 0; j < 8; j++)
        set_color(j, 0, 0, 0);

    while(0)
    {
        for(j = 0; j < 24; j++)
            set_led(j, 1);
        _delay_ms(100);
        for(j = 0; j < 24; j++)
            set_led(j, 0);
        _delay_ms(100);
    }

    if (1)
    {
        for(j = 0; j < 8; j++)
            set_color(j, 255, 0, 0);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 0, 255, 0);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 0, 0, 255);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 255, 0, 255);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 0, 255, 255);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 255, 255, 0);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 255, 255, 255);
        delay(250);
        for(j = 0; j < 8; j++)
            set_color(j, 0, 0, 0);
        delay(250);
        delay(250);
    }

    while(0)
    {
        set_color(0, 0x99, 0, 0x99);
        _delay_ms(100);
        set_color(0, 0xFF, 0xCC, 0);
        _delay_ms(100);
    }

    if (1)
    {
        for(i = 0; i < 255; i++)
        {
            for(j = 0; j < 8; j++)
                set_color(j, i, 0, 0);
            delay(10);
        }
        for(i = 0; i < 255; i++)
        {
            for(j = 0; j < 8; j++)
                set_color(j, 255, 0, i);
            delay(10);
        }
        for(i = 0; i < 255; i++)
        {
            for(j = 0; j < 8; j++)
                set_color(j, 255, i, 255);
            delay(10);
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

    for(j = 0; j < 8; j++)
        set_color(i, 0, 0, 0);

    delay(255);

	return 0;
}
#endif
