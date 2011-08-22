#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#include "debug.h"
#include "ballz.h"
#include "fsm.h"


#define ADC_START         0  
#define ADC_Z             0
#define ADC_Y             2
#define ADC_X             1
#define ADC_END           2  

#define MID_POINT   1.650
#define MID_POINT_X 1.6375
#define MID_POINT_Y 1.648
#define MID_POINT_Z 1.222
#define SENSITIVITY  .362  // V/g

// Each ADC step = this many volts
#define ADC_SENS .0025

#define X_OFFSET 608.0
#define Y_OFFSET 644.0
#define Z_OFFSET 531.0

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


// clock globals
volatile uint32_t ticks = 0;

ISR (TIMER0_OVF_vect)
{
    ticks++;
}

ISR (TIMER2_OVF_vect)
{
}

// How often we read ADC values, measured in ticks
#define CLOCK_PERIOD 5 

void get_accel(vector *a, vector *da, float *t)
{
    uint8_t         hi, low, i;
    uint32_t        temp;
    static uint32_t last_tick = 0;
    static          vector last = { 0.0, 0.0, 0.0 };

    do
    {
        cli();
        temp = ticks;
        sei();
        if (last_tick == 0)
            last_tick = temp;
    }
    while(temp - last_tick < CLOCK_PERIOD);

    for(i = ADC_START; i <= ADC_END; i++)
    {
        ADMUX &= 0xF8;
        ADMUX |= i;
        ADCSRA |= (1<<ADSC);

        while((ADCSRA & (1<<ADSC)) != 0)
            ;

        low = ADCL;
        hi = ADCH;
        if (i == ADC_X)
            a->x = (float)((hi << 8) | low);
        else
        if (i == ADC_Y)
            a->y = (float)((hi << 8) | low);
        else
        if (i == ADC_Z)
            a->z = (float)((hi << 8) | low);
    }
    last_tick = temp;
 
    *t = (float)temp * .002048;
    //dprintf("%d %d %d\n", (uint16_t)a->x, (uint16_t)a->y, (uint16_t)a->z);
    a->x = ((float)a->x - X_OFFSET) * ADC_SENS / SENSITIVITY;
    a->y = ((float)a->y - Y_OFFSET) * ADC_SENS / SENSITIVITY;
    a->z = ((float)a->z - Z_OFFSET) * ADC_SENS / SENSITIVITY;
    da->x = a->x - last.x;
    da->y = a->y - last.y;
    da->z = a->z - last.z;

    last.x = a->x;
    last.y = a->y;
    last.z = a->z;

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
    // Unused for now
    //TCCR2B |= _BV(CS21); // | _BV(CS21);// | _BV(CS20);
	//TCNT2 = 0;
    //TIMSK2 |= _BV(TOIE2);

    // Time for the clock
    //TCCR0B |= _BV(CS02); // clock / 256 / 256 = 122Hz = .008192ms per tick
    TCCR0B |= _BV(CS01) | _BV(CS00); // clock / 65 / 256 = 422Hz = .002048ms per tick
	TCNT0 = 0;
    TIMSK0 |= _BV(TOIE0);
}

void save_period(float period)
{
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

void red_leds(uint8_t state)
{
    if (state)
    {
        cbi(PORTC, 7);
        cbi(PORTC, 4);
        cbi(PORTC, 1);
        cbi(PORTD, 6);
        cbi(PORTD, 3);
        cbi(PORTB, 6);
        cbi(PORTB, 3);
        cbi(PORTB, 0);
    }
    else
    {
        sbi(PORTC, 7);
        sbi(PORTC, 4);
        sbi(PORTC, 1);
        sbi(PORTD, 6);
        sbi(PORTD, 3);
        sbi(PORTB, 6);
        sbi(PORTB, 3);
        sbi(PORTB, 0);
    }
}

void green_leds(uint8_t state)
{
    if (state)
    {
        cbi(PORTA, 7);
        cbi(PORTC, 5);
        cbi(PORTC, 2);
        cbi(PORTD, 7);
        cbi(PORTD, 4);
        cbi(PORTB, 7);
        cbi(PORTB, 4);
        cbi(PORTB, 1);
    }
    else
    {
        sbi(PORTA, 7);
        sbi(PORTC, 5);
        sbi(PORTC, 2);
        sbi(PORTD, 7);
        sbi(PORTD, 4);
        sbi(PORTB, 7);
        sbi(PORTB, 4);
        sbi(PORTB, 1);
    }
}

void blue_leds(uint8_t state)
{
    if (state)
    {
        cbi(PORTA, 6);
        cbi(PORTC, 6);
        cbi(PORTC, 3);
        cbi(PORTC, 0);
        cbi(PORTD, 5);
        cbi(PORTD, 2);
        cbi(PORTB, 5);
        cbi(PORTB, 2);
    }
    else
    {
        sbi(PORTA, 6);
        sbi(PORTC, 6);
        sbi(PORTC, 3);
        sbi(PORTC, 0);
        sbi(PORTD, 5);
        sbi(PORTD, 2);
        sbi(PORTB, 5);
        sbi(PORTB, 2);
    }
}

uint16_t EEMEM _ee_period;
float          _period;
float get_period(void)
{
    uint16_t temp;

    if (_period != 0.0)
        return _period; 

    // Storing float values in EEPROM seems sketchy since
    // the default data in EEPROM maps to nan float values.
    // Lets use uint16_t instead!
    temp = eeprom_read_word(&_ee_period);
    if (temp == 0xFFFF)
        _period = 0.0;
    else
        _period = (float)temp / 1000.0;

    return _period;
}

void set_period(float t)
{
    uint16_t temp;

    _period = t;
    temp = (uint16_t)(t * 1000.0);
    eeprom_update_word(&_ee_period, temp);
}

// Fix fuse bit: avrdude -p m324a -P usb -c avrispmkII -U hfuse:w:0xD1:m
int main(void)
{
    serial_init();
    adc_setup();
    timer_setup();

    DDRA |= (1 << PA6) | (1 << PA7);
    DDRB = 0xFF;
    DDRC = 0xFF;
    DDRD |= (1 << PD0) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);

    flash_led();
    dprintf("look at my ballz!\n");

    sei();
    fsm_loop();
#if 0
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        dprintf("%f %f %f\n", a.x, a.y, a.z);
        _delay_ms(100);
    }
#endif
	return 0;
}
