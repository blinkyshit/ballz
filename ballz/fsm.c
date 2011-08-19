#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include "debug.h"
#include "ballz.h"
#include "fsm.h"

#define STATE_START              0
#define STATE_ZERO_POINT         1
#define STATE_PERIOD_FINDER      2
#define STATE_IDLE               3
#define STATE_PULL_UP            4
#define STATE_SWINGING           5

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

// filter_reg is external storage for our filter state, order controls the strength, val is input
float lowpass(float *filter_reg, int order, float val)
{
    float filter_shift = 1 << order;
    *filter_reg = *filter_reg - (*filter_reg / filter_shift) + val;
    return *filter_reg / filter_shift;
}


void process_data_peaks(vector *a, vector *da, float t)
{
}

void process_data_zero_point(vector *a, vector *da, float t)
{
}

void process_data_period_finder(vector *a, vector *da, float t)
{
}

static float   t_last_zero_cross = 0.0;
static uint8_t moveToPullUp = 0;
static uint8_t moveToIdle = 0;
void process_data_idle(vector *a, vector *da, float t)
{
    if ((a->y > 0.0 && a->y - da->y <= 0.0) || (a->y < 0.0 && a->y - da->y >= 0.0))
        t_last_zero_cross = t;

    if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a->y) > PULL_UP_THRESHOLD_Y)
        moveToPullUp = 1;

}

//float    history[NUM_PULL_UP_HISTORY_POINTS], avg;
//uint8_t  count = 0, index = 0, i;
void process_data_pull_up(vector *a, vector *da, float t)
{
    // If we're seeing zero crossings, then the ball is near idle again
    if (t - t_last_zero_cross < PULL_UP_THRESHOLD_T)
        moveToIdle = 1;

    if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a->y) > .2)
    {
        cbi(PORTC, 1);
        sbi(PORTC, 2);
        sbi(PORTC, 3);
    }
    if (t - t_last_zero_cross > 1.0 && fabs(a->y) > .4)
    {
        sbi(PORTC, 1);
        cbi(PORTC, 2);
        sbi(PORTC, 3);
    }

    // Calculate a short running average of the recent rates of change
    // If a steep rate of change is discovered, transition to the
    // initial free fall state.
//    history[index++] = a.y;
//    index %= NUM_PULL_UP_HISTORY_POINTS;
//    count = min(count + 1, NUM_PULL_UP_HISTORY_POINTS);
//
//    if (count == NUM_PULL_UP_HISTORY_POINTS)
//        dprintf("%f\n", fabs(history[index] - history[(index-1) % count]));
}

void process_data_swinging(vector *a, vector *da, float t)
{
}

uint8_t state_zero_point(uint8_t prev_state, float t)
{
    return TRANSITION_PERIOD_FINDER;
}

uint8_t state_period_finder(uint8_t prev_state, float t)
{
    return TRANSITION_SWINGING;
}

uint8_t state_idle(uint8_t prev_state, float t)
{
    if (prev_state != STATE_IDLE)
    {
        dprintf("State: idle\n");
        // Turn on only the blue LED
        sbi(PORTC, 1);
        sbi(PORTC, 2);
        cbi(PORTC, 3);
    }
    if (moveToPullUp)
    {
        moveToPullUp = 0;
        return TRANSITION_PULL_UP;
    }

//    if (t - t_last_oob > IDLE_THRESHOLD_T)
//        break;

    return TRANSITION_NO_CHANGE;
}

uint8_t state_pull_up(uint8_t prev_state, float t)
{
    if (prev_state != STATE_PULL_UP)
    {
        dprintf("State: pull up\n");
        // Turn on only the red LED to start
        cbi(PORTC, 1);
        sbi(PORTC, 2);
        sbi(PORTC, 3);
    }

    if (moveToIdle)
    {
        // Turn off blue LED and move to pull up
        moveToIdle = 0;
        return TRANSITION_IDLE;
    }

#if 0

    if (fabs(a.x) > IDLE_THRESHOLD || fabs(a.y) > IDLE_THRESHOLD || fabs(a.z) > IDLE_THRESHOLD)
    {
        t_last_oob = t;
        dprintf("non idle: %f %f %f\n", t, fabs(a.y), fabs(a.z));
    }
    if (count == NUM_PULL_UP_HISTORY_POINTS && 
        fabs(history[index] - history[(index-1) % count] > INITIAL_FALL_SLOPE_THRESHOLD))
    {
        sbi(PORTC, 1);
        sbi(PORTC, 2);
        sbi(PORTC, 3);

        return TRANSITION_SWINGING;
    }

#endif
    return TRANSITION_NO_CHANGE;
}

uint8_t state_swinging(uint8_t prev_state, float t)
{
    return TRANSITION_IDLE;
}

void fsm_loop(void)
{
    vector  a, da;
    float   t;
    uint8_t state = STATE_ZERO_POINT;
    uint8_t prev_state = STATE_START, trans = TRANSITION_NO_CHANGE, i;

//    t_last_zero_cross = 0.0;

    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &da, &t);
        if (!updated)
            continue;

        process_data_peaks(&a, &da, t);
        process_data_zero_point(&a, &da, t);
        process_data_period_finder(&a, &da, t);
        process_data_idle(&a, &da, t);
        process_data_pull_up(&a, &da, t);
        process_data_swinging(&a, &da, t);

        switch(state)
        {
            case STATE_ZERO_POINT:
                trans = state_zero_point(prev_state, t);
                break;

            case STATE_PERIOD_FINDER:
                trans = state_period_finder(prev_state, t);
                break;

            case STATE_IDLE:
                trans = state_idle(prev_state, t);
                break;

            case STATE_PULL_UP:
                trans = state_pull_up(prev_state, t);
                break;

            case STATE_SWINGING:
                trans = state_swinging(prev_state, t);
                break;
        }

        prev_state = state;
        if (trans == TRANSITION_NO_CHANGE)
            continue;

        for(i = 0; i < NUM_TRANSITIONS; i++)
        {
            if (transition_table[i].old_state == state && transition_table[i].transition == trans)
            {
                state = transition_table[i].new_state;
                break;
            }
        }
    }
}
