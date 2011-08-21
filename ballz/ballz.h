#ifndef __BALLZ_H__
#define __BALLZ_H__

typedef struct 
{
    float x, y, z;
} vector;

uint8_t get_accel(vector *a, vector *da, float *t);
void    red_leds(uint8_t state);
void    blue_leds(uint8_t state);
void    green_leds(uint8_t state);

// Bit manipulation macros
#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

#define __unused    __attribute__((unused))

#endif
