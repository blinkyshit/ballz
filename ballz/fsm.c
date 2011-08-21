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

#define NUM_TRANSITIONS 8
Transition transition_table[NUM_TRANSITIONS] = 
{
    { STATE_ZERO_POINT,            TRANSITION_PERIOD_FINDER,  STATE_PERIOD_FINDER         },
    { STATE_PERIOD_FINDER,         TRANSITION_SWINGING,       STATE_IDLE              },
//    { STATE_PERIOD_FINDER,         TRANSITION_SWINGING,       STATE_SWINGING              },
    { STATE_SWINGING,              TRANSITION_IDLE,           STATE_IDLE                  },
    { STATE_SWINGING,              TRANSITION_PULL_UP,        STATE_PULL_UP               },
    { STATE_IDLE,                  TRANSITION_ZERO_POINT,     STATE_ZERO_POINT            },
    { STATE_IDLE,                  TRANSITION_PULL_UP,        STATE_PULL_UP               },
    { STATE_PULL_UP,               TRANSITION_IDLE    ,       STATE_IDLE                  },
    { STATE_PULL_UP,               TRANSITION_SWINGING,       STATE_SWINGING              }
};

// The lowpass filter constants for our main data stream and for our DC offset removal
#define LOWPASS_K           5
#define LOWPASS_DC_FILTER_K 10

// TODO: Document me
#define PULL_UP_THRESHOLD_Y .2
#define PULL_UP_THRESHOLD_T .5

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

void subv(vector *a, vector *subtract)
{
    a->x -= subtract->x;
    a->y -= subtract->y;
    a->z -= subtract->z;
}

#define peaks_to_keep 6
float peak_time[peaks_to_keep];
unsigned char peak_counter;

void process_data_peaks(uint8_t state, vector *a, vector *da, float t)
{
    static float last_val = 0.0, last_deriv = 0.0;
    static int looking = 0;
//    static float min = 0.0, max = 0.0, mid = 0.0;
    
    if (state != STATE_PERIOD_FINDER && state != STATE_SWINGING)
        return;

    // midpoint cross for position
//    if (sign(last_val - mid) - sign(a->z - mid))
    if (sign(last_val) - sign(a->z))
        looking = 1;

    // check for zero cross of derivative (ie, a peak in position)
    if (sign(last_deriv) - sign(da->z))
    {
/*
        if (last_deriv > da->z)
            min = a->z;
        else
            max = a->z;
        mid = (min+max)/2.0;
*/        
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
        //dprintf("period: %f (%f)\n", period_ball, deviation);
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
    static float last_dy = 0.0;

    if ((a->y > 0.0 && a->y - da->y <= 0.0) || (a->y < 0.0 && a->y - da->y >= 0.0))
        t_last_zero_cross = t;

    if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a->y) > PULL_UP_THRESHOLD_Y)
        moveToPullUp = 1;

    last_dy = da->y;
}

// If the absolute value of the derivative of the Z axis
// goes above this threshold, we will consider the user to have let go of the ball.
#define SWING_START_DERIV_THRESHOLD_Z .005
void process_data_pull_up(uint8_t state, vector *a, vector *da, float t)
{
    // If we're seeing zero crossings, then the ball is near idle again
    // if (t - t_last_zero_cross < PULL_UP_THRESHOLD_T)
    //    moveToIdle = 1;

    if (state == STATE_PULL_UP)
    {
        if (t - t_last_zero_cross > PULL_UP_THRESHOLD_T && fabs(a->y) > .2)
        {
            red_leds(1);
            green_leds(0);
            blue_leds(0);
        }
        if (t - t_last_zero_cross > 1.0 && fabs(a->y) > .4)
        {
            red_leds(0);
            green_leds(1);
            blue_leds(0);
        }
        if (fabs(da->z) > SWING_START_DERIV_THRESHOLD_Z)
            moveToSwinging = 1;
    }
}

void process_data_swinging(uint8_t state, vector *a, vector *da, float t)
{
    static float idle_check_reg = 100.0;
    float        idle_check;
    static int   counter = 0;

    if (state != STATE_SWINGING)
    {
        idle_check_reg = 100.0;
        return;
    }

    idle_check = lowpass(&idle_check_reg, 10, a->z * a->z);
    if (idle_check < .0001)
        moveToIdle = 1;

    ++counter;
    if ((counter & 0x3f) == 0 || moveToIdle)
        dprintf("%f %f\n", t, idle_check);
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
        red_leds(0);
        green_leds(0);
        blue_leds(1);
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
        dprintf("State: pull up\n");
        // Turn on only the red LED to start
        red_leds(1);
        green_leds(0);
        blue_leds(0);
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
        dprintf("State: swinging\n");
        red_leds(1);
        green_leds(0);
        blue_leds(1);
    }

    if (moveToIdle)
    {
        moveToIdle = 0;
        return TRANSITION_IDLE;
    }
    if (moveToPullUp)
    {
        moveToPullUp = 0;
        return TRANSITION_PULL_UP;
    }

    return TRANSITION_NO_CHANGE;
}

void fsm_loop(void)
{
    vector  a, da, a_dc, a_no_dc;
    vector  a_lp_reg = {0, 0, 0}, da_lp_reg = {0, 0, 0}, a_dc_reg = {0, 0, 0};
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

//        dprintf("%f %f %f\n", t, a.z, da.z);

        // Replace a and da with lowpass filtered versions of the same
        lowpassv(&a_lp_reg, LOWPASS_K, &a);
        lowpassv(&da_lp_reg, LOWPASS_K, &da);

        a_no_dc = a;
        // if we're past the initial pull back impulse, then try to find center of swinging
        if (state == STATE_PERIOD_FINDER || state == STATE_SWINGING)
        {
            // Estimate the DC offset of a, then subtract that from a_no_dc to center the data
            a_dc = a;
            lowpassv(&a_dc_reg, LOWPASS_DC_FILTER_K, &a_dc);
            subv(&a_no_dc, &a_dc);
        }
        
//        dprintf("%f %f %f\n", t, a.y, da.y);

        process_data_peaks(STATE_SWINGING, &a_no_dc, &da, t);
        process_data_zero_point(state, &a, &da, t);
        process_data_period_finder(state, &a_no_dc, &da, t);
        process_data_idle(state, &a, &da, t);
        process_data_pull_up(state, &a, &da, t);
        process_data_swinging(state, &a_no_dc, &da, t);

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
