/*
 * encoder.c  – Quadrature encoder reader (ATmega32U4, no timers)
 * Counts ?/? on every edge of channel-A,
 * direction is decided by the logic level on channel-B.
 *
 * Author : Endeavor360
 * Date   : 21-May-2025
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "encoder.h"

/* --------------------- private state --------------------- */
static volatile int32_t left_cnt  = 0;
static volatile int32_t right_cnt = 0;

/* ========================================================= */
void encoder_init(void)
{
    /* -------- LEFT encoder pins -------- */
    ENC_L_A_DDR  &= ~_BV(ENC_L_A_BIT);      /* input          */
    ENC_L_B_DDR  &= ~_BV(ENC_L_B_BIT);
    ENC_L_A_PORT |=  _BV(ENC_L_A_BIT);      /* enable pull-up */
    ENC_L_B_PORT |=  _BV(ENC_L_B_BIT);

    /* Any-edge interrupt on INT6 (PE6) */
    EICRB |=  _BV(ISC60);   /* ISC61:0 = 01 ? any logical change        */
    EICRB &= ~_BV(ISC61);
    EIMSK |=  _BV(INT6);    /* enable INT6                              */

    /* -------- RIGHT encoder pins -------- */
    ENC_R_A_DDR  &= ~_BV(ENC_R_A_BIT);
    ENC_R_B_DDR  &= ~_BV(ENC_R_B_BIT);
    ENC_R_A_PORT |=  _BV(ENC_R_A_BIT);
    ENC_R_B_PORT |=  _BV(ENC_R_B_BIT);

    /* Any-edge interrupt on INT0 (PB0) */
    EICRA |=  _BV(ISC00);   /* ISC01:0 = 01 ? any logical change        */
    EICRA &= ~_BV(ISC01);
    EIMSK |=  _BV(INT0);    /* enable INT0                              */
}

/* ------------------------- ISR LEFT ---------------------- */
ISR(INT6_vect)
{
    /* Read channel-B – direction decision */
    if (ENC_L_B_PINREG & _BV(ENC_L_B_BIT))
        left_cnt--;   /* CW ? */
    else
        left_cnt++;   /* CCW ? */
}

/* ------------------------- ISR RIGHT --------------------- */
ISR(INT0_vect)
{
    if (ENC_R_B_PINREG & _BV(ENC_R_B_BIT))
        right_cnt--;  /* CW */
    else
        right_cnt++;  /* CCW */
}

/* ------------------------- API --------------------------- */
int32_t encoder_get_left(void)   { int32_t c; cli(); c = left_cnt;  sei(); return c; }
int32_t encoder_get_right(void)  { int32_t c; cli(); c = right_cnt; sei(); return c; }

void    encoder_reset_left(void) { cli(); left_cnt  = 0; sei(); }
void    encoder_reset_right(void){ cli(); right_cnt = 0; sei(); }
