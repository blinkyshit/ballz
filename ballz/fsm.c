#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include "debug.h"
#include "ballz.h"
#include "fsm.h"

#define STATE_IDLE                  1
#define STATE_PULL_UP               2
#define STATE_INITIAL_FALL          3
#define STATE_SWINGING              4

#define TRANSITION_PULL_UP       1
#define TRANSITION_IDLE          2
#define TRANSITION_INITIAL_FALL  3
#define TRANSITION_SWINGING      4

typedef struct 
{
    uint8_t old_state;
    uint8_t transition;
    uint8_t new_state;
} Transition;

#define NUM_TRANSITIONS 5
Transition transition_table[NUM_TRANSITIONS] = 
{
    { STATE_IDLE,                  TRANSITION_PULL_UP,        STATE_PULL_UP               },

    { STATE_PULL_UP,               TRANSITION_INITIAL_FALL,   STATE_INITIAL_FALL          },
    { STATE_PULL_UP,               TRANSITION_IDLE    ,       STATE_IDLE                  },

    { STATE_INITIAL_FALL,          TRANSITION_SWINGING,       STATE_SWINGING              },

    { STATE_SWINGING,              TRANSITION_IDLE,           STATE_IDLE                  }
};

#define PULL_UP_THRESHOLD_Y .2
#define PULL_UP_THRESHOLD_T .5

static vector last;
static float  t, t_last_zero_cross = 0.0;

#define SENSITIVITY  .362  // V/g
#define ADC_SENS .0025
#define X_OFFSET 655
#define Y_OFFSET 627
#define Z_OFFSET 511

uint8_t state_idle(void)
{
    vector a;
    float t;

    dprintf("state: idle\n");

    // Turn on only the blue LED
    sbi(PORTC, 1);
    sbi(PORTC, 2);
    cbi(PORTC, 3);
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a.y) > PULL_UP_THRESHOLD_Y)
        {
            // Turn off blue LED and move to pull up
            sbi(PORTC, 1);
            sbi(PORTC, 2);
            sbi(PORTC, 3);
   
            return TRANSITION_PULL_UP;
        }

        if ((a.y > 0.0 && last.y <= 0.0) || (a.y < 0.0 && last.y >= 0.0))
            t_last_zero_cross = t;

        last.x = a.x; last.y = a.y; last.z = a.z;
    }
    return TRANSITION_PULL_UP;
}

uint8_t state_pull_up(void)
{
    vector a;
    float t;

    dprintf("state: pull up\n");
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        // If we've seen a zero crossing in the last sec, turn off
        if (t - t_last_zero_cross < PULL_UP_THRESHOLD_T)
        {
            sbi(PORTC, 1);
            sbi(PORTC, 2);
            sbi(PORTC, 3);

            return TRANSITION_IDLE;
        }
        if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a.y) > .2)
        {
            cbi(PORTC, 1);
            sbi(PORTC, 2);
            sbi(PORTC, 3);
        }
        if (t - t_last_zero_cross > 1.0 && fabs(a.y) > .4)
        {
            sbi(PORTC, 1);
            cbi(PORTC, 2);
            sbi(PORTC, 3);
        }

        if ((a.y > 0.0 && last.y <= 0.0) || (a.y < 0.0 && last.y >= 0.0))
            t_last_zero_cross = t;

        //_delay_ms(50);
        last.x = a.x; last.y = a.y; last.z = a.z;
    }

    return TRANSITION_INITIAL_FALL;
}


uint8_t state_initial_fall(void)
{
    dprintf("state: initial fall\n");
    return TRANSITION_SWINGING;
}

uint8_t state_swinging(void)
{
    vector a;
    float t;

    dprintf("state: swinging\n");
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        // If we've seen a zero crossing in the last sec, turn off
        if (t - t_last_zero_cross < PULL_UP_THRESHOLD_T)
        {
            sbi(PORTC, 1);
            sbi(PORTC, 2);
            sbi(PORTC, 3);

            return TRANSITION_IDLE;
        }
        if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a.y) > .2)
        {
            cbi(PORTC, 1);
            sbi(PORTC, 2);
            sbi(PORTC, 3);
        }
        if (t - t_last_zero_cross > 1.0 && fabs(a.y) > .4)
        {
            sbi(PORTC, 1);
            cbi(PORTC, 2);
            sbi(PORTC, 3);
        }

        if ((a.y > 0.0 && last.y <= 0.0) || (a.y < 0.0 && last.y >= 0.0))
            t_last_zero_cross = t;

        //_delay_ms(50);
        last.x = a.x; last.y = a.y; last.z = a.z;
    }

    return TRANSITION_IDLE;
}

void fsm_loop(void)
{
    uint8_t state = STATE_IDLE;
    uint8_t new_state = 0, trans, i;

    last.x = last.y = last.z = 0.0;
    t_last_zero_cross = 0.0;

    trans = state_idle();
    for(;;)
    {
        for(i = 0; i < NUM_TRANSITIONS; i++)
        {
            if (transition_table[i].old_state == state && transition_table[i].transition == trans)
            {
                new_state = transition_table[i].new_state;
                break;
            }
        }

        switch(new_state)
        {
            case STATE_IDLE:
                trans = state_idle();
                break;

            case STATE_PULL_UP:
                trans = state_pull_up();
                break;

            case STATE_INITIAL_FALL:
                trans = state_initial_fall();
                break;

            case STATE_SWINGING:
                trans = state_swinging();
                break;
        }
        state = new_state;
    }
}
