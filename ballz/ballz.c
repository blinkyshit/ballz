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
#define ADC_Z             0
#define ADC_Y             1
#define ADC_X             2
#define ADC_END           3  

#define MID_POINT   1.650
#define MID_POINT_X 1.6375
#define MID_POINT_Y 1.648
#define MID_POINT_Z 1.222
#define SENSITIVITY  .362 

// Each ADC step = this many volts
#define ADC_SENS .0025

#define X_OFFSET 655
#define Y_OFFSET 627
#define Z_OFFSET 511

/* Accelerometer info: MMA7331L

   362mV/g output @ 3.3V
    Offset:
    x 1.607
    y 1.683
    z 1.285

   Vcc A0 A1 A2 [unused]

        Read   computed
   A0-Z 1.238  1.297
   A1-Y 1.550  1.550
   A2-X 1.688  1.63

   A0-Z 1.235  1.28     12% error
   A1-Y 1.648  1.5675   22% error
   A2-X 1.597  1.5675    8% error

   A0-Z 1.222  1.28 
   A1-Y 1.648  1.56
   A2-X 1.690  1.6375 

*/

typedef struct 
{
    float x, y, z;
} vector;

// ADC globals
volatile uint8_t  adc_index = ADC_START;
volatile uint16_t accel_x, accel_y, accel_z = 0;
volatile int8_t   data_updated = 0;

// clock globals
volatile uint32_t ticks = 0;

ISR (TIMER0_OVF_vect)
{
    ticks++;
}

ISR (TIMER2_OVF_vect)
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
        accel_x = (float)((hi << 8) | low);
    }
    else
    if (adc_index == ADC_Y)
    {
        accel_y = (float)((hi << 8) | low);
    }
    else
    if (adc_index == ADC_Z)
    {
        accel_z = (float)((hi << 8) | low);
        data_updated++;
    }

    adc_index++;
    if (adc_index == ADC_END)
        adc_index = ADC_START;
}

uint8_t get_accel(vector *v, float *t)
{
    uint8_t u;
    uint16_t x, y, z;
    uint32_t temp;

    cli();
    if (!data_updated)
    {
        sei();
        return 0;
    }

    x = accel_x;
    y = accel_y;
    z = accel_z;
    u = data_updated;
    data_updated = 0;
    temp = ticks;
    sei();

    v->x = x;
    v->y = y;
    v->z = z;

    *t = (float)temp * .008192;
    return u;
}


void adc_setup(void)
{
    //ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
    ADCSRA = (1 << ADPS1); // | (1 << ADPS0); 
    ADMUX = (1<<REFS0) | (1<< REFS1); 
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
    // ADC conversion completion check timer
    TCCR2B |= _BV(CS21); // | _BV(CS21);// | _BV(CS20);
	TCNT2 = 0;
    TIMSK2 |= _BV(TOIE2);

    TCCR0B |= _BV(CS02);
	TCNT0 = 0;
    TIMSK0 |= _BV(TOIE0);
}

void flash_led(void)
{
    uint8_t i, j;

    for(j = 0; j < 5; j++)
    {
        cbi(PORTA, 6);
        cbi(PORTA, 7);
        for(i = 0; i < 8; i++)
        {
            cbi(PORTB, i);
            cbi(PORTC, i);
        }
        cbi(PORTD, 0);
        cbi(PORTD, 2);
        cbi(PORTD, 3);
        cbi(PORTD, 4);
        cbi(PORTD, 5);
        cbi(PORTD, 6);
        cbi(PORTD, 7);
        _delay_ms(75); 

        sbi(PORTA, 6);
        sbi(PORTA, 7);
        for(i = 0; i < 8; i++)
        {
            sbi(PORTB, i);
            sbi(PORTC, i);
        }
        sbi(PORTD, 0);
        sbi(PORTD, 2);
        sbi(PORTD, 3);
        sbi(PORTD, 4);
        sbi(PORTD, 5);
        sbi(PORTD, 6);
        sbi(PORTD, 7);
        _delay_ms(75); 
    }
}

int main(void)
{
    vector a, dg, n, last;
    int i;
    float m, t;

    serial_init();
    adc_setup();
    timer_setup();

    DDRA |= (1 << PA6) | (1 << PA7);
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD |= (1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);

    flash_led();

    dprintf("look at my ballz!\n");
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        dg.x = (a.x - X_OFFSET) * ADC_SENS * SENSITIVITY;
        dg.y = (a.y - Y_OFFSET) * ADC_SENS * SENSITIVITY;
        dg.z = (a.z - Z_OFFSET) * ADC_SENS * SENSITIVITY;

        //dprintf("r: %f, %f, %f\n", a.x, a.y, a.z);
        //dprintf("v: %f, %f, %f\n", v.x, v.y, v.z);
        //dprintf("g: %f, %f, %f |%f|\n\n", g.x, g.y, g.z, m);
        dprintf("%f %f\n", t, dg.x);

//        for(i = 0; i < 10; i++)
//        _delay_ms(100);
    }

	return 0;
}
