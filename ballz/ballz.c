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

#define ADC_START         0  
#define ADC_Y             0
#define ADC_X             1
#define ADC_END           2  

#define MAX_ACCEL_VALUE 0

typedef struct 
{
    float x, y;
} vector;

volatile uint8_t adc_index = ADC_START;
volatile int16_t accel_x;
volatile int16_t accel_y;

ISR (TIMER1_OVF_vect)
{
    if ((ADCSRA & (1<<ADSC)) != 0)
        return;

    ADMUX &= 0xF8;
    ADMUX |= adc_index;
    ADCSRA |= (1<<ADSC)|(1<<ADIE);
}

ISR(ADC_vect)
{
    uint8_t hi, low;

    low = ADCL;
    hi = ADCH;

    if (adc_index == ADC_X)
    {
        accel_x = MAX_ACCEL_VALUE - ((hi << 8) | low);
    }
    else
    if (adc_index == ADC_Y)
    {
        accel_y = MAX_ACCEL_VALUE - ((hi << 8) | low);
    }

    adc_index++;
    if (adc_index == ADC_END)
        adc_index = ADC_START;
}

void get_accel(int16_t *x, int16_t *y)
{
    cli();
    *x = accel_x;
    *y = accel_y;
    sei();
}


void adc_setup(void)
{
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
    ADMUX = (1<<REFS0); // | (1<< REFS1); 
    ADCSRA |= (1<<ADEN); 
}

void adc_shutdown(void)
{
    ADCSRA &= ~(1<<ADEN);
}

uint8_t adc_read(uint8_t ch)
{
    uint8_t val, dummy;

    ADMUX &= 0xF8;
    ADMUX |= ch;

    ADCSRA |= (1<<ADSC);
    while(ADCSRA & 0b01000000);
    dummy = ADCL;
    val = ADCH;
    ADCSRA &= ~(1<<ADSC);
    return ADCH;
}

void timer_setup(void)
{
    TCCR1B |= _BV(CS12) | _BV(CS11);// | _BV(CS20);
	TCNT1 = 0;
    TIMSK |= _BV(TOIE1);
}

void flash_led(void)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        cbi(PORTD, 6);
        _delay_ms(100); 
        sbi(PORTD, 6);
        _delay_ms(100); 
    }
}

int main(void)
{
    int8_t i;
    int16_t x, y, last_x, last_y;

    serial_init();
    adc_setup();
    timer_setup();

    DDRD |= (1 << PD6);
    flash_led();

    dprintf("accel test start\n");
    last_x = last_y = 0;
    while(1)
    {
        get_accel(&x, &y);
        dprintf("%d, %d\n", x - last_x, y - last_y);
        last_x = x; last_y = y;
        _delay_ms(100);
    }

	return 0;
}
