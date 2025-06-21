/* =========================================================
 * encoder.c  – Robust quadrature-encoder reader (ATmega32U4)
 * 4× quadrature decoder with mixed INT/PCINT
 *
 * Author : Endeavor360
 * Date   : 25-May-2025
 * ========================================================= */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "encoder.h"

/* private state */
static volatile int32_t left_cnt, right_cnt;
static volatile uint8_t left_last_state, right_last_state;

/* forward */
static inline void enc_handle(volatile int32_t *cnt, volatile uint8_t *last, uint8_t a, uint8_t b)
{
	uint8_t encoded    = (a<<1) | b;
	uint8_t trans      = (*last<<2) | encoded;
	switch (trans) {
		case 0b1101: case 0b0100:
		case 0b0010: case 0b1011:  (*cnt)--; break;
		case 0b1110: case 0b0111:
		case 0b0001: case 0b1000:  (*cnt)++; break;
		default: /* bounce/invalid */ break;
	}
	*last = encoded;
}

/* Initialise both encoders + emergency pin */
void encoder_init(void)
{
	/*------------- LEFT: PD2/PD3 as INT0/INT1 -------------*/
	ENC_L_A_DDR  &= ~_BV(ENC_L_A_BIT);
	ENC_L_B_DDR  &= ~_BV(ENC_L_B_BIT);
	ENC_L_A_PORT |=  _BV(ENC_L_A_BIT);
	ENC_L_B_PORT |=  _BV(ENC_L_B_BIT);

	/* INT0 on PD2, any edge */
	EICRA  |=  _BV(ISC00);
	EICRA  &= ~_BV(ISC01);
	EIMSK  |=  _BV(INT0);
	/* INT1 on PD3, any edge */
	EICRA  |=  _BV(ISC10);
	EICRA  &= ~_BV(ISC11);
	EIMSK  |=  _BV(INT1);

	/* Seed last_state */
	left_last_state = ((ENC_L_A_PINREG & _BV(ENC_L_A_BIT))?2:0)
	| ((ENC_L_B_PINREG & _BV(ENC_L_B_BIT))?1:0);

	/*------------- RIGHT: PB6/PB7 + EMG PB3 as PCINT0 -------------*/
	/* pins input + pull-up */
	ENC_R_A_DDR    &= ~_BV(ENC_R_A_BIT);
	ENC_R_B_DDR    &= ~_BV(ENC_R_B_BIT);
	EMG_BTN_DDR    &= ~_BV(EMG_BTN_BIT);
	ENC_R_A_PORT   |=  _BV(ENC_R_A_BIT);
	ENC_R_B_PORT   |=  _BV(ENC_R_B_BIT);
	EMG_BTN_PORT   |=  _BV(EMG_BTN_BIT);

	/* enable PCINT[7:0] */
	PCICR         |=  _BV(PCIE0);
	/* mask PB3, PB6, PB7 */
	PCMSK0        |=  _BV(EMG_BTN_BIT)
	| _BV(ENC_R_A_BIT)
	| _BV(ENC_R_B_BIT);

	right_last_state = ((ENC_R_A_PINREG & _BV(ENC_R_A_BIT))?2:0)
	| ((ENC_R_B_PINREG & _BV(ENC_R_B_BIT))?1:0);

	/* Zero counters */
	left_cnt  = right_cnt = 0;
}

/* ------------ LEFT ISRs (INT0 & INT1) ------------- */
ISR(INT0_vect)
{
	uint8_t a = (ENC_L_A_PINREG & _BV(ENC_L_A_BIT))?1:0;
	uint8_t b = (ENC_L_B_PINREG & _BV(ENC_L_B_BIT))?1:0;
	enc_handle(&left_cnt, &left_last_state, a, b);
}

ISR(INT1_vect)
{
	uint8_t a = (ENC_L_A_PINREG & _BV(ENC_L_A_BIT))?1:0;
	uint8_t b = (ENC_L_B_PINREG & _BV(ENC_L_B_BIT))?1:0;
	enc_handle(&left_cnt, &left_last_state, a, b);
}

/* ------------ RIGHT + EMERGENCY (PCINT0) ------------- */
ISR(PCINT0_vect)
{
	/* 1) emergency check */
	if (!(EMG_BTN_PINREG & _BV(EMG_BTN_BIT))) {
		/* button is pressed (active-low) */
		// TODO: call emergency handler here
		// emergency_button_pressed();
	}

	/* 2) right encoder decode */
	uint8_t a = (ENC_R_A_PINREG & _BV(ENC_R_A_BIT))?1:0;
	uint8_t b = (ENC_R_B_PINREG & _BV(ENC_R_B_BIT))?1:0;
	enc_handle(&right_cnt, &right_last_state, a, b);
}

/* =========== public API (unchanged) =========== */
int32_t encoder_get_left(void)
{ cli(); int32_t c = left_cnt; sei(); return c; }

int32_t encoder_get_right(void)
{ cli(); int32_t c = right_cnt; sei(); return c; }

void encoder_reset_left(void)
{ cli(); left_cnt = 0; sei(); }

void encoder_reset_right(void)
{ cli(); right_cnt = 0; sei(); }