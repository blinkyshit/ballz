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

// clock globals
volatile uint32_t ticks = 0;

ISR (TIMER0_OVF_vect)
{
    ticks++;
}

float get_time(void)
{
    uint32_t temp;

    cli();
    temp = ticks;
    sei();

    return (float)temp * .002048;
} 

void timer_setup(void)
{
    // Time for the clock
    //TCCR0B |= _BV(CS02); // clock / 256 / 256 = 122Hz = .008192ms per tick
    TCCR0B |= _BV(CS01) | _BV(CS00); // clock / 65 / 256 = 422Hz = .002048ms per tick
    TCNT0 = 0;
    TIMSK0 |= _BV(TOIE0);
}

void flash_led(void)
{
    uint8_t i;

    for(i = 0; i < 5; i++)
    {
        sbi(PORTD, 2);
        _delay_ms(100); 
        cbi(PORTD, 2);
        _delay_ms(100); 
    }
}

void flash_led_fuss(void)
{
    uint8_t i;

    for(i = 0;; i++)
    {
        sbi(PORTB, 5);
        _delay_ms(100); 
        cbi(PORTB, 5);
        _delay_ms(100); 
    }
}

void adc_setup(void)
{
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
    ADMUX= (1<<REFS0);  
    ADMUX |= (1<<ADLAR);  
    ADCSRA |= (1<<ADEN); 
}

void adc_shutdown(void)
{
    ADCSRA &= ~(1<<ADEN);
}

uint8_t get_light_level(void)
{
    uint8_t val, dummy;

    sbi(PORTB, 1);
    adc_setup();

    ADCSRA |= (1<<ADSC);
    while(ADCSRA & 0b01000000);
    dummy = ADCL;
    val = ADCH;
    ADCSRA &= ~(1<<ADSC);

    adc_shutdown();
    cbi(PORTB, 1);

    return val;
}

uint8_t light_state = 0;

uint8_t are_lights_on(void)
{
    return light_state;
}

void turn_ballz_on(void)
{
    sbi(PORTB, 5);
    cbi(PORTB, 4);
    cbi(PORTD, 6);
    light_state = 1;
}

void turn_ballz_off(void)
{
    cbi(PORTB, 5);
    sbi(PORTB, 4);
    sbi(PORTD, 6);
    light_state = 0;
}

#define LIGHT_THRESHOLD 50
//#define BALL_RESET_PERIOD 180 // seconds
#define BALL_RESET_PERIOD    180 
#define BALL_RESET_DURATION    2

int main(void)
{
    uint8_t  i, j, l;
    float    last_reset = 0.0, t, ball_restart_time = 0.0;

    serial_init();
    adc_setup();
    timer_setup();

    // 12V in: 
    // Green: Ground
    // Blue: 3.3V
    // PA0 / A0: light sensor read (Orange)
    // PB1 / 9: light sensor enable
    // PB5 / 13: the green LED on the breakout board. It indicates if ballz should be on or not
    // PD2: the red LED that shows the board is working by flashing 5 times
    // PD6: the orange safety light ON/OFF switch
    DDRB |= (1 << PB4) | (1 << PB1) | (1 << PB5);
    DDRD |= (1 << PD6) | (1 << PD2) | (1 << PD3);

    dprintf("I am the master! Where are my minions?\n");
    turn_ballz_off();
    flash_led();
    //flash_led_fuss();

    sei();

    while(1)
    {
        l = get_light_level();
        if (l > LIGHT_THRESHOLD && are_lights_on())
        {
            dprintf("lights off!\n");
            turn_ballz_off();
        }
        if (l < LIGHT_THRESHOLD && !are_lights_on())
        {
            dprintf("light on!\n");
            turn_ballz_on();
        }
        if (!are_lights_on())
        {
            for(i = 0; i < 10; i++)
               _delay_ms(100);
            continue;
        }

        // its dark. lights are on!
        t = get_time();
        if (last_reset == 0.0)
            last_reset = t;

        if (t - last_reset > BALL_RESET_PERIOD)
        {
            //dprintf("%f: Ballz off!\n", t);
            ball_restart_time = t + BALL_RESET_DURATION;
            last_reset = t;
        }

        if (ball_restart_time > 0 && t > ball_restart_time)
        {
            //dprintf("%f: Ballz on!\n", t);
            ball_restart_time = 0.0;
        }
#if 0
        for(i = 0; i < 2; i++)
        {
            sbi(PORTD, 4);
            _delay_ms(100);
            cbi(PORTD, 4);
            _delay_ms(100);

            for(j = 0; j < 3; j++)
            {
                sbi(PORTD, 4);
                _delay_ms(40);
                cbi(PORTD, 4);
                _delay_ms(40);
            }

            sbi(PORTD, 4);
            _delay_ms(100);
            cbi(PORTD, 4);
            _delay_ms(100);
        }
#endif
    }

	return 0;
}
