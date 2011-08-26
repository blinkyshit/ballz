#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include "pwm.h"
#include "hue.h"

#include "debug.h"

void rainbow(void)
{
    uint8_t i, j;
    color_t c;

    while(1)
        for(i = 0; i < HUE_MAX; i++)
        {
             color_hue(i, &c);
             for(j = 0; j < 8; j++)
                 set_color(j, c.red, c.green, c.blue);
             _delay_ms(2);
        }
}

void rainbow_circle(void)
{
    uint8_t i, j;
    color_t c;

    while(1)
        for(i = 0; i < HUE_MAX; i+=2)
        {
             for(j = 0; j < 8; j++)
             {
                 color_hue((i + j * 32) % HUE_MAX, &c);
                 set_color(j, c.red, c.green, c.blue);
             }
             wdt_reset(); 
             _delay_ms(10);
        }
}
