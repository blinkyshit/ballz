#include <avr/io.h>
#include <math.h>
#include <util/delay.h>
#include "debug.h"
#include "ballz.h"
#include "fsm.h"

enum BallState
{
    BALL_AT_REST                  = 0,
    BALL_PULL_UP,
    BALL_RELEASE,
    BALL_SWINGING,
    BALL_UPSIDE_DOWN,
    BALL_PLAYA_BADGER
};

// The lowpass filter constants for our main data stream and for our DC offset removal
#define LOWPASS_K           3
#define LOWPASS_DC_FILTER_K 6

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

#define PEAKS_TO_KEEP 6
float peak_time[PEAKS_TO_KEEP];
unsigned char peak_counter;
unsigned char last_peak_side = 0, start_peak_side = -1;

void process_data_peaks(uint8_t state, vector *a, vector *da, float t)
{
    static float last_val = 0.0, last_deriv = 0.0;
    static int looking = 0;
    
    //if (state != STATE_PERIOD_FINDER && state != STATE_SWINGING)
    if (state != BALL_SWINGING)
        return;

    if (sign(last_val) - sign(a->z))
        looking = 1;

    // check for zero cross of derivative (ie, a peak in position)
    if (sign(last_deriv) - sign(da->z))
    {
        if (looking)
        {
            int i;

            looking = 0;
            for (i = PEAKS_TO_KEEP - 1; i > 0; i--)
                peak_time[i] = peak_time[i - 1];
            peak_time[0] = t;
            peak_counter++;
            last_peak_side = sign(a->y);
        }
    }
    last_val = a->z;
    last_deriv = da->z;
}

void process_data_zero_point(uint8_t state, vector *a, vector *da, float t)
{
}

// deviation tolerance is an empirical number 'proof by eyeball'
#define         PERIOD_JITTER_TOLERANCE 0.2
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
    for (i = 0; i < PEAKS_TO_KEEP - 2; i++)
        deviation += fabs(peak_time[i] - 2*peak_time[i+1] + peak_time[i+2]) / (PEAKS_TO_KEEP-2);

//    ++counter;
//    if ((counter & 0x3f) == 0)
//        dprintf("p:%f %f\n", period_ball, deviation);
    if (deviation < PERIOD_JITTER_TOLERANCE)
    {
        period_ball = (peak_time[0] - peak_time[PEAKS_TO_KEEP - 1]) / (PEAKS_TO_KEEP - 1);
        period_ball *= 4; 		 // Z axis peak to -peak occur twice as fast as x or y
        if (period_reg < 0.1) 	 // first sample? Then prime the pump with our current value
            period_reg = 2 * period_ball; // 2 = 1 << (filter order = 1) to adjust for final scaling
        period_ball = lowpass(&period_reg, 1, period_ball);
        period_data_valid = 1;
        save_period(period_ball);
        dprintf("period: %f (%f)\n", period_ball, deviation);
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

// returns the phase as a value from [0, 1)
float swing_phase(float t)
{
    // subtract current time from last peak time, divide by the period, return the fractional part
    // slightly complicated as global peak array is on z, which is twice as fast as the period
    
    float phase = (t - peak_time[0]) / period_ball;
    if (last_peak_side != start_peak_side)
        phase += 0.5;
    
    // add a static offset to make up for the lag, 0.1s?
    
    // return fractional part of phase;
    return phase;
}

void state_zero_point(uint8_t prev_state, float t)
{
}

void state_period_finder(uint8_t prev_state, float t)
{
}

void state_idle(uint8_t prev_state, float t)
{
}

void state_pull_up(uint8_t prev_state, float t)
{
}

void state_swinging(uint8_t prev_state, float t)
{
}

#define ZERO_CROSSING_COUNTS 20
uint8_t is_idle(vector *a, vector *da, float t)
{
    static float cross_time[ZERO_CROSSING_COUNTS] = {0,}, last_dz = 0.0;
    int i;
    
    i = sign(last_dz) - sign(da->z);
    last_dz = da->z;
    
    if (i)
    {   // we have a crossing
        for (i=ZERO_CROSSING_COUNTS - 1; i>0; i--)
            cross_time[i] = cross_time[i-1];
        cross_time[0] = t;
    }

    if (t - cross_time[ZERO_CROSSING_COUNTS - 1] < 1.0)
        if (fabs(a->y) < 0.3)
            return 1;

    return 0;
}

float integral_ay(vector *a, vector *da, float t)
{
    static float accum = 0.0, accum_reg = 0.0;
    accum = lowpass(accum_reg, 2, accum + a->y);
    return accum;
}

uint8_t is_pullup(vector *a, vector *da, float t)
{
    // Ball periods range between 1.45 and 2.5 seconds. Z is half that.
    // if dz is positive for more than 1/4 of the max ball period, then
    // we're being pulled back in one of the directions
    
    static float first_positive_time = 0.0;
    
    if (integral_ay(a, da, t) < 0.2)
    {
        first_positive_time = 0.0;
        return 0;
    }
    
    if (first_positive_time < 0.0001) // testing for time = 0.0
        first_positive_time = t;
    if (t - first_positive_time > 0.25 * 3)
        return 1;
    return 0;
}

enum BallState infer_behavior(vector *a, vector *da, float t)
{
    // examines a, da, t, and possibly past behavior to determine a best guess for what's
    // currently happening with the ball. If none of the conditions fire, return the last state.

    static enum BallState last = BALL_AT_REST;
    static float last_change_t = 0.0;

    if (a->z > 1.0)
    {
        last = BALL_UPSIDE_DOWN;
        last_change_t = t;
    }
    
    if (last != BALL_RELEASE && is_idle(a, da, t))
    {
        last = BALL_AT_REST;
        last_change_t = t;
    }
    
    if ( last == BALL_PULL_UP && da->y > 0.02)
    {
        last = BALL_RELEASE;
        last_change_t = t;
    }
    
    if (is_pullup(a, da, t))
    {
        last = BALL_PULL_UP;
        last_change_t = t;
    }
    
    if (0)
    {
        last = BALL_PLAYA_BADGER;
        last_change_t = t;
    }

//    if (period_data_valid || (last == BALL_RELEASE && (t - last_change_t > 0.5)) )
    if ((last == BALL_RELEASE && (t - last_change_t > 0.5)) )
    {
        last = BALL_SWINGING;
        last_change_t = t;
    }
    
    return last;
}

void fsm_loop(void)
{
    vector  a, da, a_dc, a_no_dc;
    vector  a_lp_reg = {0, 0, 0}, da_lp_reg = {0, 0, 0}, a_dc_reg = {0, 0, 0};
    float   t;
    enum BallState current_ball_state = -1, previous_ball_state;

    while(1)
    {
        get_accel(&a, &da, &t);

//        dprintf("%f %f %f\n", t, a.z, da.z);

        // Replace a and da with lowpass filtered versions of the same
        lowpassv(&a_lp_reg, LOWPASS_K, &a);
        lowpassv(&da_lp_reg, LOWPASS_K, &da);

        a_no_dc = a;
        // if we're past the initial pull back impulse, then try to find center of swinging
        if (current_ball_state == BALL_AT_REST || current_ball_state == BALL_RELEASE || current_ball_state == BALL_SWINGING)
        {
            // Estimate the DC offset of a, then subtract that from a_no_dc to center the data
            a_dc = a;
            lowpassv(&a_dc_reg, LOWPASS_DC_FILTER_K, &a_dc);
            subv(&a_no_dc, &a_dc);
        }
        
//        dprintf("%f %f %f\n", t, a.y, da.y);

        process_data_peaks(current_ball_state, &a_no_dc, &da, t);
        process_data_zero_point(current_ball_state, &a, &da, t);
        process_data_period_finder(current_ball_state, &a_no_dc, &da, t);

        current_ball_state = infer_behavior(&a, &da, t);

        if (current_ball_state != previous_ball_state)
            switch (current_ball_state) {
                case BALL_AT_REST: 
                    dprintf("%f: Ball at rest\n", t); 
                    red_leds(0);
                    green_leds(0);
                    blue_leds(1);
                    break;
                case BALL_PULL_UP: 
                    red_leds(1);
                    green_leds(0);
                    blue_leds(0);
                    dprintf("%f: Ball pull up\n", t); 
                    break;
                case BALL_RELEASE: 
                    dprintf("%f: Ball release\n", t); 
                    red_leds(0);
                    green_leds(1);
                    blue_leds(0);
                    break;
                case BALL_SWINGING: 
                    dprintf("%f: Ball swinging\n", t); 
                    red_leds(1);
                    green_leds(0);
                    blue_leds(1);
                    break;
                case BALL_UPSIDE_DOWN: dprintf("%f: Ball upside down\n", t); break;
                case BALL_PLAYA_BADGER: dprintf("%f: Playa badgers are screwing with us!\n", t); break;
                default: dprintf("%f: ** Unknown ball state!\n", t); break;
            }

        previous_ball_state = current_ball_state;
    }
}
