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

ISR (TIMER2_OVF_vect)
{
}


void timer_setup(void)
{
    TCCR2B |= _BV(CS22) | _BV(CS21);// | _BV(CS20);
	TCNT2 = 0;
    TIMSK2 |= _BV(TOIE2);
}

void pwm_setup(void)
{
	/* Set to Fast PWM */
	TCCR0A |= _BV(WGM01) | _BV(WGM00);

	// Set the compare output mode
	TCCR0A |= _BV(COM0A1);
	TCCR0A |= _BV(COM0B1);

	// Reset timers and comparators
	OCR0A = 0;
	OCR0B = 0;
	TCNT0 = 0;

    // Set the clock source
	TCCR0B |= _BV(CS00);

    // Set PWM pins as outputs
    DDRD |= (1<<PD6)|(1<<PD5)|(1<<PD3);
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
        sbi(PORTD, 4);
        _delay_ms(100); 
        cbi(PORTD, 4);
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

void turn_lights_on(void)
{
    sbi(PORTB, 5);
    cbi(PORTB, 4);
    light_state = 1;
}

void turn_lights_off(void)
{
    cbi(PORTB, 5);
    sbi(PORTB, 4);
    light_state = 0;
}

#define LIGHT_THRESHOLD 50

int main(void)
{
    uint8_t  i, j, l, state = 0;

    serial_init();
    adc_setup();
    timer_setup();

    // 12V in: 
    // Green: Ground
    // Blue: 3.3V
    // PA0 / A0: light sensor read (Orange)
    // PB1 / 9: light sensor enable
    // PB5 / 13: the green LED on the breakout board. It indicates if lights should be on or not
    // PD2: the red LED that shows the board is working by flashing 5 times
    // PD3: the safety light ON/OFF switch
    DDRB |= (1 << PB4) | (1 << PB1) | (1 << PB5);
    DDRD |= (1 << PD2) | (1 << PD3);

    turn_lights_off();
    flash_led();

    sei();

    while(1)
    {
        l = get_light_level();
        if (l > LIGHT_THRESHOLD && are_lights_on())
        {
            turn_lights_off();
        }
        if (l < LIGHT_THRESHOLD && !are_lights_on())
        {
            turn_lights_on();
        }
        if (!are_lights_on())
        {
            //for(i = 0; i < 10; i++)
               _delay_ms(100);
            continue;
        }
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
    }

	return 0;
}
