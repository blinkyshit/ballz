#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include "debug.h"
#include "ballz.h"
#include "fsm.h"

#define STATE_ZERO_POINT            0
#define STATE_PERIOD_FINDER         1
#define STATE_IDLE                  2
#define STATE_PULL_UP               3
#define STATE_SWINGING              4

#define TRANSITION_NO_CHANGE     0
#define TRANSITION_ZERO_POINT    1
#define TRANSITION_PERIOD_FINDER 2
#define TRANSITION_IDLE          3
#define TRANSITION_PULL_UP       4
#define TRANSITION_SWINGING      5

typedef struct 
{
    uint8_t old_state;
    uint8_t transition;
    uint8_t new_state;
} Transition;

#define NUM_TRANSITIONS 7
Transition transition_table[NUM_TRANSITIONS] = 
{
    { STATE_ZERO_POINT,            TRANSITION_PERIOD_FINDER,  STATE_PERIOD_FINDER         },
    { STATE_PERIOD_FINDER,         TRANSITION_SWINGING,       STATE_SWINGING              },
    { STATE_SWINGING,              TRANSITION_IDLE,           STATE_IDLE                  },
    { STATE_IDLE,                  TRANSITION_ZERO_POINT,     STATE_ZERO_POINT            },
    { STATE_IDLE,                  TRANSITION_PULL_UP,        STATE_PULL_UP               },
    { STATE_PULL_UP,               TRANSITION_IDLE    ,       STATE_IDLE                  },
    { STATE_PULL_UP,               TRANSITION_SWINGING,       STATE_SWINGING              }
};

#define PULL_UP_THRESHOLD_Y .2
#define PULL_UP_THRESHOLD_T .5
#define IDLE_THRESHOLD      .15
#define IDLE_THRESHOLD_T    1.00 

#define NUM_PULL_UP_HISTORY_POINTS    16
#define INITIAL_FALL_SLOPE_THRESHOLD  .1

static vector last;
static float  t, t_last_zero_cross = 0.0;

void process_data_peaks(vector *v, float t)
{
}

void process_zero_point(vector *v, float t)
{
}

void process_period_finder(vector *v, float t)
{
}

void process_data_idle(vector *v, float t)
{
}

void process_data_pull_up(vector *v, float t)
{
}

void process_data_swinging(vector *v, float t)
{
}

uint8_t state_zero_point(void)
{
    return TRANSITION_PERIOD_FINDER;
}

uint8_t state_period_finder(void)
{
    return TRANSITION_SWINGING;
}

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
    vector   a;
    float    t, history[NUM_PULL_UP_HISTORY_POINTS], avg;
    uint8_t  count = 0, index = 0, i;

    dprintf("state: pull up\n");
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;


        // Calculate a short running average of the recent rates of change
        // If a steep rate of change is discovered, transition to the
        // initial free fall state.
        history[index++] = a.y;
        index %= NUM_PULL_UP_HISTORY_POINTS;
        count = min(count + 1, NUM_PULL_UP_HISTORY_POINTS);
    
        if (count == NUM_PULL_UP_HISTORY_POINTS)
            dprintf("%f\n", fabs(history[index] - history[(index-1) % count]));
        if (count == NUM_PULL_UP_HISTORY_POINTS && 
            fabs(history[index] - history[(index-1) % count] > INITIAL_FALL_SLOPE_THRESHOLD))
        {
            sbi(PORTC, 1);
            sbi(PORTC, 2);
            sbi(PORTC, 3);

            return TRANSITION_SWINGING;
        }

        // If we're seeing zero crossings, then the ball is near idle again
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

}

uint8_t state_swinging(void)
{
    vector a;
    float t, t_last_oob = 0.0;

    dprintf("state: swinging\n");
    cbi(PORTC, 1);
    sbi(PORTC, 2);
    cbi(PORTC, 3);
    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        if (fabs(a.x) > IDLE_THRESHOLD || fabs(a.y) > IDLE_THRESHOLD || fabs(a.z) > IDLE_THRESHOLD)
        {
            t_last_oob = t;
            dprintf("non idle: %f %f %f\n", t, fabs(a.y), fabs(a.z));
        }

        if (t - t_last_oob > IDLE_THRESHOLD_T)
            break;

        last.x = a.x; last.y = a.y; last.z = a.z;
    }

    sbi(PORTC, 1);
    sbi(PORTC, 2);
    sbi(PORTC, 3);

    return TRANSITION_IDLE;
}

void fsm_loop(void)
{
    vector  a;
    float   t, t_last_oob = 0.0;
    uint8_t state = STATE_IDLE;
    uint8_t new_state = 0, trans, i;

    last.x = last.y = last.z = 0.0;
    t_last_zero_cross = 0.0;

    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &t);
        if (!updated)
            continue;

        process_data_peaks(&a, t);
        process_zero_point(&a, t);
        process_period_finder(&a, t);
        process_data_idle(&a, t);
        process_data_pull_up(&a, t);
        process_data_swinging(&a, t);

        switch(state)
        {
            case STATE_ZERO_POINT:
                trans = state_zero_point();
                break;

            case STATE_PERIOD_FINDER:
                trans = state_period_finder();
                break;

            case STATE_IDLE:
                trans = state_idle();
                break;

            case STATE_PULL_UP:
                trans = state_pull_up();
                break;

            case STATE_SWINGING:
                trans = state_swinging();
                break;
        }

        if (trans == TRANSITION_NO_CHANGE)
            continue;

        for(i = 0; i < NUM_TRANSITIONS; i++)
        {
            if (transition_table[i].old_state == state && transition_table[i].transition == trans)
            {
                new_state = transition_table[i].new_state;
                break;
            }
        }
        state = new_state;
    }
}
