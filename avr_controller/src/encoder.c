/* =========================================================
 * encoder.c   Robust quadrature-encoder reader (ATmega32U4)
 * 4 quadrature decoder with mixed INT/PCINT
 *
 * Author : Endeavor360
 * Date   : 25-May-2025
 * ========================================================= */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/atomic.h>
#include "config.h"
#include "encoder.h"

/* private state */
static volatile int32_t left_cnt, right_cnt;
static volatile uint8_t left_last_state, right_last_state;

static int32_t prev_left_cnt = 0;
static int32_t prev_right_cnt = 0;

static int32_t left_delta = 0;
static int32_t right_delta = 0;

static float fwd_change_mm = 0.0f;
static float rot_change_deg = 0.0f;

static float robot_distance_mm = 0.0f;
static float robot_angle_deg = 0.0f;

static uint64_t prev_ts_us = 0;
static uint32_t loop_dt_us = 1;

/* ---------- helpers ---------- */
static inline float inv_dt(void) { return 1e6f / (float)loop_dt_us; }

static volatile bool emg_flag = false;

/* forward */
static inline void enc_handle(volatile int32_t *cnt, volatile uint8_t *last, uint8_t a, uint8_t b)
{
	uint8_t encoded = (a << 1) | b;
	uint8_t trans = (*last << 2) | encoded;
	switch (trans)
	{
	case 0b1101:
	case 0b0100:
	case 0b0010:
	case 0b1011:
		(*cnt)--;
		break;
	case 0b1110:
	case 0b0111:
	case 0b0001:
	case 0b1000:
		(*cnt)++;
		break;
	default: /* bounce/invalid */
		break;
	}
	*last = encoded;
}

/* Initialise both encoders + emergency pin */
void encoder_init(void)
{
	/*------------- LEFT: PD2/PD3 as INT2/INT3 -------------*/
	ENC_L_A_DDR &= ~_BV(ENC_L_A_BIT);
	ENC_L_B_DDR &= ~_BV(ENC_L_B_BIT);
	ENC_L_A_PORT |= _BV(ENC_L_A_BIT);
	ENC_L_B_PORT |= _BV(ENC_L_B_BIT);

	/* INT2 on PD2, any edge */
	EICRA |= _BV(ISC20);
	EICRA &= ~_BV(ISC21);
	EIMSK |= _BV(INT2);

	/* INT3 on PD3, any edge */
	EICRA |= _BV(ISC30);
	EICRA &= ~_BV(ISC31);
	EIMSK |= _BV(INT3);

	/* Seed last_state */
	left_last_state = ((ENC_L_A_PINREG & _BV(ENC_L_A_BIT)) ? 2 : 0) | ((ENC_L_B_PINREG & _BV(ENC_L_B_BIT)) ? 1 : 0);

	/*------------- RIGHT: PE6 as INT6, PB4 as PCINT4 and EMG BTN as PCINT7 -------------*/
	/* pins input + pull-up */
	ENC_R_B_DDR &= ~_BV(ENC_R_B_BIT);
	ENC_R_B_PORT |= _BV(ENC_R_B_BIT);

	ENC_R_A_DDR &= ~_BV(ENC_R_A_BIT);
	EMG_BTN_DDR &= ~_BV(EMG_BTN_BIT);
	ENC_R_A_PORT |= _BV(ENC_R_A_BIT);
	EMG_BTN_PORT |= _BV(EMG_BTN_BIT);

	/* INT6 on PE6 any edge */
	EICRB |= _BV(ISC60);  // sense on either edge
	EICRB &= ~_BV(ISC61); // (leave ISC61 = 0)
	EIMSK |= _BV(INT6);	  // enable INT6

	/* enable PCINT[7:0] */
	PCICR |= _BV(PCIE0);
	/* mask PB4, PB7 */
	PCMSK0 |= _BV(ENC_R_A_BIT) | _BV(EMG_BTN_BIT);

	right_last_state = ((ENC_R_A_PINREG & _BV(ENC_R_A_BIT)) ? 2 : 0) | ((ENC_R_B_PINREG & _BV(ENC_R_B_BIT)) ? 1 : 0);

	/* Zero counters */
	left_cnt = right_cnt = 0;

	encoder_odometry_reset();
}

/* ------------ LEFT ISRs (INT2 & INT3) ------------- */
ISR(INT2_vect)
{
	uint8_t a = (ENC_L_A_PINREG & _BV(ENC_L_A_BIT)) ? 1 : 0;
	uint8_t b = (ENC_L_B_PINREG & _BV(ENC_L_B_BIT)) ? 1 : 0;
	enc_handle(&left_cnt, &left_last_state, a, b);
}

ISR(INT3_vect)
{
	uint8_t a = (ENC_L_A_PINREG & _BV(ENC_L_A_BIT)) ? 1 : 0;
	uint8_t b = (ENC_L_B_PINREG & _BV(ENC_L_B_BIT)) ? 1 : 0;
	enc_handle(&left_cnt, &left_last_state, a, b);
}

ISR(INT6_vect)
{
	uint8_t a = (ENC_R_A_PINREG & _BV(ENC_R_A_BIT)) ? 1 : 0;
	uint8_t b = (ENC_R_B_PINREG & _BV(ENC_R_B_BIT)) ? 1 : 0;
	enc_handle(&right_cnt, &right_last_state, a, b);
}

/* ------------ RIGHT + EMERGENCY (PCINT0) ------------- */
ISR(PCINT0_vect)
{
	/* 1) emergency check */
	if (!(EMG_BTN_PINREG & _BV(EMG_BTN_BIT)))
	{
		emg_flag = true;
	}

	/* 2) right encoder decode */
	uint8_t a = (ENC_R_A_PINREG & _BV(ENC_R_A_BIT)) ? 1 : 0;
	uint8_t b = (ENC_R_B_PINREG & _BV(ENC_R_B_BIT)) ? 1 : 0;
	enc_handle(&right_cnt, &right_last_state, a, b);
}

/* =========== public API (unchanged) =========== */
int32_t encoder_get_left(void)
{
	cli();
	int32_t c = left_cnt;
	sei();
	return c;
}

int32_t encoder_get_right(void)
{
	cli();
	int32_t c = right_cnt;
	sei();
	return c;
}

void encoder_reset_left(void)
{
	cli();
	left_cnt = 0;
	sei();
}

void encoder_reset_right(void)
{
	cli();
	right_cnt = 0;
	sei();
}

void encoder_reset_both(void)
{
	encoder_reset_left();
	encoder_reset_right();
}

bool encoder_emergency_hit(void)
{
	bool hit;
	cli(); /* atomic: read-then-clear */
	hit = emg_flag;
	// emg_flag = false; // needs to stop all operations in a way that restart can fix it
	sei();
	return hit;
}

void encoder_odometry_reset(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		prev_left_cnt = left_cnt = 0;
		prev_right_cnt = right_cnt = 0;

		fwd_change_mm = rot_change_deg = 0.0f;
		robot_distance_mm = robot_angle_deg = 0.0f;

		prev_ts_us = micros64(); /* ← use global timer0 based time-base */
		loop_dt_us = 1;
	}
}

void encoder_odometry_update(void)
{
	uint64_t now_us = micros64();
	loop_dt_us = (uint32_t)(now_us - prev_ts_us);
	if (loop_dt_us == 0)
		loop_dt_us = 1;
	prev_ts_us = now_us;

	/* snapshot counts atomically */
	int32_t l, r;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		l = left_cnt;
		r = right_cnt;
	}
	left_delta = l - prev_left_cnt;
	right_delta = r - prev_right_cnt;
	prev_left_cnt = l;
	prev_right_cnt = r;

	const float mm_per_pulse = MM_PER_ROTATION / (4.0f * ENCODER_PPR * GEAR_RATIO);
	float left_mm = left_delta * mm_per_pulse;
	float right_mm = right_delta * mm_per_pulse;

	fwd_change_mm = 0.5f * (left_mm + right_mm);
	rot_change_deg = (right_mm - left_mm) * DEG_PER_MM_DIFF;

	robot_distance_mm += fwd_change_mm;
	robot_angle_deg += rot_change_deg;
}

float encoder_left_speed_mm_s(void) { return left_delta * (MM_PER_ROTATION / (4.0f * ENCODER_PPR * GEAR_RATIO)) * inv_dt(); }
float encoder_right_speed_mm_s(void) { return right_delta * (MM_PER_ROTATION / (4.0f * ENCODER_PPR * GEAR_RATIO)) * inv_dt(); }
float encoder_robot_speed_mm_s(void) { return fwd_change_mm * inv_dt(); }
float encoder_robot_omega_dps(void) { return rot_change_deg * inv_dt(); }
float encoder_robot_distance_mm(void) { return robot_distance_mm; }
float encoder_robot_angle_deg(void) { return robot_angle_deg; }
uint32_t encoder_loop_time_us(void) { return loop_dt_us; }