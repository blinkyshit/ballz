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
    { STATE_PERIOD_FINDER,         TRANSITION_SWINGING,       STATE_IDLE              },
//    { STATE_PERIOD_FINDER,         TRANSITION_SWINGING,       STATE_SWINGING              },
    { STATE_SWINGING,              TRANSITION_IDLE,           STATE_IDLE                  },
    { STATE_IDLE,                  TRANSITION_ZERO_POINT,     STATE_ZERO_POINT            },
    { STATE_IDLE,                  TRANSITION_PULL_UP,        STATE_PULL_UP               },
    { STATE_PULL_UP,               TRANSITION_IDLE    ,       STATE_IDLE                  },
    { STATE_PULL_UP,               TRANSITION_SWINGING,       STATE_SWINGING              }
};

#define PULL_UP_THRESHOLD_Y .2
#define PULL_UP_THRESHOLD_T .5

#define NUM_PULL_UP_HISTORY_POINTS    16
#define SWINGING_IDLE_ZERO_CROSSING_THRESHOLD_T .5

int sign(float val)
{
    if (val > 0)
    	return 1;
    return -1;
}

// filter_reg is external storage for our filter state, order controls the strength, val is input
float lowpass(float *filter_reg, int order, float val)
{
    float filter_shift = 1 << order;
    *filter_reg = *filter_reg - (*filter_reg / filter_shift) + val;
    return *filter_reg / filter_shift;
}

// lowpass a 3-vector, overwrites v with the result
void lowpassv(vector *lp_reg, int order, vector *v)
{
    v->x = lowpass( &lp_reg->x, order, v->x );
    v->y = lowpass( &lp_reg->y, order, v->y );
    v->z = lowpass( &lp_reg->z, order, v->z );
}

#define peaks_to_keep 6
float peak_time[peaks_to_keep];
unsigned char peak_counter;

void process_data_peaks(uint8_t state, vector *a, vector *da, float t)
{
    static float last_val = 0.0, last_deriv = 0.0;
    static int looking = 0;
    static float min = 0.0, max = 0.0, mid = 0.0;
    
    if (state != STATE_PERIOD_FINDER && state != STATE_SWINGING)
        return;

    // midpoint cross for position
    if (sign(last_val - mid) - sign(a->z - mid))
        looking = 1;

    // check for zero cross of derivative (ie, a peak in position)
    if (sign(last_deriv) - sign(da->z))
    {
        if (last_deriv > da->z)
            min = a->z;
        else
            max = a->z;
        mid = (min+max)/2.0;
        
        if (looking)
        {
            int i;

            looking = 0;
            for (i = peaks_to_keep - 1; i > 0; i--)
                peak_time[i] = peak_time[i - 1];
            peak_time[0] = t;
            peak_counter++;
        }
    }
    last_val = a->z;
    last_deriv = da->z;
}

void process_data_zero_point(uint8_t state, vector *a, vector *da, float t)
{
}

// deviation tolerance is an empirical number 'proof by eyeball'
#define         period_jitter_tolerance 0.1
float           period_ball;
unsigned char   period_data_valid;

void process_data_period_finder(uint8_t state, __unused vector *a, __unused vector *da, __unused float t)
{
    static unsigned char last_peak_counter = 0;
    int i;
    float deviation = 0.0;
    static float period_reg = 0.0;

    // Exit if there are no new peaks to process
    if (last_peak_counter == peak_counter)
        return;

    last_peak_counter = peak_counter;

    // compute the average of: difference of the differences: ([i]-[i+1]) - ([i+1]-[i+2]) 
    for (i = 0; i < peaks_to_keep - 2; i++)
        deviation += fabs(peak_time[i] - 2*peak_time[i+1] + peak_time[i+2]) / (peaks_to_keep-2);

    if (deviation < period_jitter_tolerance)
    {
        period_ball = (peak_time[0] - peak_time[peaks_to_keep - 1]) / (peaks_to_keep - 1);
        period_ball *= 4; 		 // Z axis peak to -peak occur twice as fast as x or y
        if (period_reg < 0.1) 	 // first sample? Then prime the pump with our current value
            period_reg = 2 * period_ball; // 2 = 1 << (filter order = 1) to adjust for final scaling
        period_ball = lowpass(&period_reg, 1, period_ball);
        period_data_valid = 1;
        dprintf("period: %f\n", period_ball);
        //printf("%f %f %f %f\n", t, a->z, da->z, period_ball);
    }
    else
    {
        // we're being played with, reset our state
        period_reg = 0.0;
        // FIXME: We almost certainly don't want to do this, but leaving it in until
        // I can talk with Rob.
        period_data_valid = 0;
    }
}

static float   t_last_zero_cross = 0.0;
static uint8_t moveToPullUp = 0;
static uint8_t moveToIdle = 0;
static uint8_t moveToSwinging = 0;
void process_data_idle(uint8_t state, vector *a, vector *da, float t)
{
    if ((a->y > 0.0 && a->y - da->y <= 0.0) || (a->y < 0.0 && a->y - da->y >= 0.0))
        t_last_zero_cross = t;

    if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a->y) > PULL_UP_THRESHOLD_Y)
        moveToPullUp = 1;

    //dprintf("%f %f %f\n", t, a->y, da->y);
}

void process_data_pull_up(uint8_t state, vector *a, vector *da, float t)
{
    // If we're seeing zero crossings, then the ball is near idle again
    if (t - t_last_zero_cross < PULL_UP_THRESHOLD_T)
        moveToIdle = 1;

    if (state == STATE_PULL_UP)
    {
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
        if (fabs(da->y) > .02)
        {
            moveToSwinging = 1;
            //dprintf("da: %f\n", da->y);
        }
    }
}

#define IDLE_ACCEL_Y .05
#define IDLE_DERIV_Y .01
#define IDLE_THRESHOLD_T 1.00 
float last_non_idle_t = 0.0;
void process_data_swinging(uint8_t state, vector *a, vector *da, float t)
{
    if (fabs(a->y) > IDLE_ACCEL_Y || fabs(da->y) > IDLE_DERIV_Y)
        last_non_idle_t = t;
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
        //dprintf("State: idle\n");
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

    return TRANSITION_NO_CHANGE;
}

uint8_t state_pull_up(uint8_t prev_state, float t)
{
    if (prev_state != STATE_PULL_UP)
    {
        //dprintf("State: pull up\n");
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
    if (moveToSwinging)
    {
        // Turn off blue LED and move to pull up
        moveToSwinging = 0;
        return TRANSITION_SWINGING;
    }

    return TRANSITION_NO_CHANGE;
}

uint8_t state_swinging(uint8_t prev_state, float t)
{
    if (prev_state != STATE_SWINGING)
    {
        //dprintf("State: swinging\n");
        cbi(PORTC, 1);
        sbi(PORTC, 2);
        cbi(PORTC, 3);
    }

    if (t - last_non_idle_t > IDLE_THRESHOLD_T)
        return TRANSITION_IDLE;

    return TRANSITION_NO_CHANGE;
}

void fsm_loop(void)
{
    vector  a, da;
    vector  a_lp_reg = {0, 0, 0}, da_lp_reg = {0, 0, 0};
    float   t;
    uint8_t state = STATE_ZERO_POINT;
    uint8_t prev_state = STATE_START, trans = TRANSITION_NO_CHANGE, i;
//    uint32_t samples = 0, done = 0;

//    t_last_zero_cross = 0.0;

    while(1)
    {
        uint8_t updated; 

        updated = get_accel(&a, &da, &t);
        if (!updated)
            continue;

//        samples++;
//        if (t > 30 && !done)
//        {
//            done = 1;
//            dprintf("Samples: %d\n", samples);
//        }

        dprintf("%f %f %f\n", t, a.z, da.z);

        // Replace a and da with lowpass filtered versions of the same
        lowpassv(&a_lp_reg, 5, &a);
        lowpassv(&da_lp_reg, 5, &da);

        process_data_peaks(STATE_SWINGING, &a, &da, t);
        process_data_zero_point(state, &a, &da, t);
        process_data_period_finder(state, &a, &da, t);
        process_data_idle(state, &a, &da, t);
        process_data_pull_up(state, &a, &da, t);
        process_data_swinging(state, &a, &da, t);

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
