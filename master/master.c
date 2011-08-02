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

    for(i = 0; i < 3; i++)
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

    DDRC |= (1 << PC1);
    adc_setup();

    ADCSRA |= (1<<ADSC);
    while(ADCSRA & 0b01000000);
    dummy = ADCL;
    val = ADCH;
    ADCSRA &= ~(1<<ADSC);

    DDRC &= ~(1 << PC1);
    adc_shutdown();
    dprintf("val: %d\n", val);

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
    light_state = 1;
}

void turn_lights_off(void)
{
    cbi(PORTB, 5);
    light_state = 0;
}

#define LIGHT_THRESHOLD 50

int main(void)
{
    uint8_t  i, l, state = 0;

    serial_init();
    adc_setup();
    timer_setup();

    // Pin A0 light sensor read
    // Pin A1 (PC1) light sensor enable
    // Pin 13 (PB5) is the on board LED
    DDRB |= (1 << PB5);

    turn_lights_off();
    flash_led();

    sei();
    dprintf("master controller starting!\n");

    while(1)
    {
        l = get_light_level();
        if (l > LIGHT_THRESHOLD && are_lights_on())
        {
            turn_lights_off();
            dprintf("state: %d\n", are_lights_on());
        }
        if (l < LIGHT_THRESHOLD && !are_lights_on())
        {
            turn_lights_off();
        }
        dprintf("l: %d, state: %d\n", l, are_lights_on());
        for(i = 0; i < 10; i++)
            _delay_ms(200);
    }

	return 0;
}
