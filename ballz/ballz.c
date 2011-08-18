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
#define ADC_X             1
#define ADC_Y             2
#define ADC_END           3  

#define WINDOW_SIZE       10

#define MID_POINT   1.650
#define MID_POINT_X 1.648
#define MID_POINT_Y 1.691
#define MID_POINT_Z 1.226
//#define MID_POINT_X 1.5100 1.648
//#define MID_POINT_Y 1.6075 1.691
//#define MID_POINT_Z 1.6895 1.226
#define SENSITIVITY  .362 

/* Accelerometer info: MMA7331L

   362mV/g output @ 3.3V
    Offset:
    x 1.607
    y 1.683
    z 1.285

*/

typedef struct 
{
    float x, y, z;
} vector;

// ADC globals
volatile uint8_t adc_index = ADC_START;
volatile vector  accel;
volatile vector  accel_win[WINDOW_SIZE];
volatile int8_t  win_count = 0, data_updated = 0;

// clock globals
//volatile uint32_t ticks = 0;

//ISR (TIMER0_OVF_vect)
//{
//    ticks++;
//}

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
        accel_win[win_count].x = (float)((hi << 8) | low);
    }
    else
    if (adc_index == ADC_Y)
    {
        accel_win[win_count].y = (float)((hi << 8) | low);
    }
    else
    if (adc_index == ADC_Z)
    {
        accel_win[win_count++].z = (float)((hi << 8) | low);

        if (win_count == WINDOW_SIZE)
        {
            uint8_t i;

            accel.x = accel.y = accel.z = 0.0;
            for(i = 0; i < WINDOW_SIZE; i++)
            {
                accel.x += accel_win[i].x;
                accel.y += accel_win[i].y;
                accel.z += accel_win[i].z;
            }
            accel.x /= WINDOW_SIZE;
            accel.y /= WINDOW_SIZE;
            accel.z /= WINDOW_SIZE;
            data_updated++;
            win_count = 0;
        }
    }

    adc_index++;
    if (adc_index == ADC_END)
        adc_index = ADC_START;
}

uint8_t get_accel(vector *v) //, float *t)
{
    uint8_t u;
//    uint32_t temp;

    cli();
    if (!data_updated)
    {
        sei();
        return 0;
    }

    v->x = accel.x;
    v->y = accel.y;
    v->z = accel.z;
    u = data_updated;
    data_updated = 0;
//    temp = ticks;
    sei();

//    *t = (float)temp * .008192;
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

//    TCCR0B |= _BV(CS02);
//	TCNT0 = 0;
//    TIMSK0 |= _BV(TOIE0);
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
    vector a, v, g, n, last;
    int8_t i, m_sign;
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

        updated = get_accel(&a); //, &t);
        if (!updated)
            continue;

        // Convert from raw values to mV
        v.x = (a.x * 2.56 / 1024);
        v.y = (a.y * 2.56 / 1024);
        v.z = (a.z * 2.56 / 1024);

        // Convert from mV to g
        g.x = (v.x - MID_POINT_X) / SENSITIVITY;
        g.y = (v.y - MID_POINT_Y) / SENSITIVITY;
        g.z = -((v.z - MID_POINT_Z) / SENSITIVITY);

        // Calculate the magnitude of the vector
        m = sqrt((g.x * g.x) + (g.y * g.y) + (g.z * g.z));

        dprintf("r: %f, %f, %f\n", a.x, a.y, a.z);
        //dprintf("v: %f, %f, %f\n\n", v.x, v.y, v.z);
        //dprintf("g: %f, %f, %f |%f|\n", g.x, g.y, g.z, m);

        n.x = g.x / m;
        n.y = g.y / m;
        n.z = g.z / m;
        //dprintf("n: %f, %f, %f\n", n.x, n.y, n.z);
        //dprintf("%f, %f, %f\n", n.x - GRAVITY_X, n.y - GRAVITY_Y, n.z - GRAVITY_Z);
        //dprintf("%f\n", n.x - GRAVITY_X);
        //dprintf("%f\n",  n.x);
        //dprintf("%f %f\n", t, n.x);
        //dprintf("%f, %f, %f\n", last.x - n.x, last.y - n.y, last.z - n.z);
        last.x = n.x; last.y = n.y; last.z = n.z;
        //_delay_ms(10);
    }

	return 0;
}
