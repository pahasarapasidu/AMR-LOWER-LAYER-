/*
 * motors.c
 *
 * Created: 5/6/2025 4:26:05 AM
 * Author: Endeavor360
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include "motors.h"
#include "config.h"

/* private state ----------------------------------------------------------- */
static uint16_t left_top;
static uint16_t right_top;

/* ----------- hardware‐assisted edge (toggle) counters & ISRs ------------   */

volatile uint32_t left_edge_cnt = 0;
volatile uint32_t right_edge_cnt = 0;

/* helpers ----------------------------------------------------------------- */
static inline uint32_t velocity_to_freq(uint16_t velocity)
{
	return ((uint32_t)velocity * STEPS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_MM * M_PI);
}

/* public functions -------------------------------------------------------- */
void motors_init(void)
{
	/* I/O direction ------------------------------------------------------- */
	LEFT_PUL_DDR |= _BV(LEFT_PUL_BIT);
	LEFT_DIR_DDR |= _BV(LEFT_DIR_BIT);
	LEFT_ENA_DDR |= _BV(LEFT_ENA_BIT);

	RIGHT_PUL_DDR |= _BV(RIGHT_PUL_BIT);
	RIGHT_DIR_DDR |= _BV(RIGHT_DIR_BIT);
	RIGHT_ENA_DDR |= _BV(RIGHT_ENA_BIT);

	/* default levels � pulses high, forward, drivers disabled ------------ */
	LEFT_PUL_PORT |= _BV(LEFT_PUL_BIT);
	LEFT_DIR_PORT |= _BV(LEFT_DIR_BIT);
	LEFT_ENA_PORT &= ~_BV(LEFT_ENA_BIT);

	RIGHT_PUL_PORT |= _BV(RIGHT_PUL_BIT);
	RIGHT_DIR_PORT |= _BV(RIGHT_DIR_BIT);
	RIGHT_ENA_PORT &= ~_BV(RIGHT_ENA_BIT);

	/* — Timer-3 (16-bit) drives LEFT motor PUL on OC3A (PC6/D5) — */
	TCCR3A = _BV(COM3A0); /* toggle OC3A on compare match          */
	TCCR3B = _BV(WGM32);  /* CTC mode (TOP = OCR3A), clk stopped   */

	/* — Timer-1 (16-bit) drives RIGHT motor PUL on OC1A (PB5/D9) — */
	TCCR1A = _BV(COM1A0); /* toggle OC1A on compare match          */
	TCCR1B = _BV(WGM12);  /* CTC mode, clk stopped                 */

	/* enable OC-compare interrupts for edge counting ------------------ */
	TIMSK3 |= _BV(OCIE3A); /* Timer-3 Compare-A                     */
	TIMSK1 |= _BV(OCIE1A); /* Timer-1 Compare-A                     */
	
	motors_enable_all(true);
}

/* ISR: each toggle = one edge ----------------------------------------- */
ISR(TIMER3_COMPA_vect)
{
	left_edge_cnt++;
}
ISR(TIMER1_COMPA_vect)
{
	right_edge_cnt++;
}

void motors_enable_left(bool en)
{
	(en ? (LEFT_ENA_PORT |= _BV(LEFT_ENA_BIT))
		: (LEFT_ENA_PORT &= ~_BV(LEFT_ENA_BIT)));
}

void motors_enable_right(bool en)
{
	(en ? (RIGHT_ENA_PORT |= _BV(RIGHT_ENA_BIT))
		: (RIGHT_ENA_PORT &= ~_BV(RIGHT_ENA_BIT)));
}

void motors_enable_all(bool en)
{
	motors_enable_left(en);
	motors_enable_right(en);
}

void motors_set_dir_left(bool fwd)
{
	(fwd ? (LEFT_DIR_PORT |= _BV(LEFT_DIR_BIT))
		 : (LEFT_DIR_PORT &= ~_BV(LEFT_DIR_BIT)));
}

void motors_set_dir_right(bool fwd)
{
	(fwd ? (RIGHT_DIR_PORT |= _BV(RIGHT_DIR_BIT))
		 : (RIGHT_DIR_PORT &= ~_BV(RIGHT_DIR_BIT)));
}

void motors_set_speed_left(uint16_t vel)
{
	uint32_t freq = velocity_to_freq(vel);
	uint32_t top = (F_CPU / (2UL * freq * CLOCK_DIVISOR_TIMER3)) - 1UL;
	if (top > 0xFFFF)
		top = 0xFFFF; /* clamp to 16-bit */

	left_top = (uint16_t)top;
	OCR3A = left_top;

	/* start Timer-3 with /1024 prescale */
	TCCR3B &= ~(_BV(CS32) | _BV(CS31) | _BV(CS30)); /* clear first   */
	TCCR3B |= PRE_SCALE_TIMER3;
}

void motors_set_speed_right(uint16_t vel)
{
	uint32_t freq = velocity_to_freq(vel);
	uint32_t top = (F_CPU / (2UL * freq * CLOCK_DIVISOR)) - 1UL;

	right_top = (uint16_t)top;
	OCR1A = right_top;

	/* start Timer-1 with /1024 pre-scale */
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); /* clear first   */
	TCCR1B |= PRE_SCALE_TIMER1;
}

void motors_set_speed_both(uint16_t vel_left, uint16_t vel_right)
{
	motors_set_speed_left(vel_left);
	motors_set_speed_right(vel_right);
}

void motors_update(motion *ctx, float velocity, float omega)
{
	/* Store the latest set-points */
	ctx->velocity = velocity;
	ctx->omega    = omega;

	/* Feed-forward terms based on desired wheel tangential speed */
	float tangent_speed = ctx->omega * WHEEL_BASE_MM * M_PI/ 360.0 ;
	float left_speed    = ctx->velocity - tangent_speed;
	float right_speed   = ctx->velocity + tangent_speed;

	if (left_speed > 0 && right_speed > 0 ){
		motors_set_dir_left(true);
		motors_set_dir_right(false);
		motors_set_speed_both(left_speed, right_speed);
	} else if (left_speed < 0 && right_speed > 0 ) {
		motors_set_dir_left(false);
		motors_set_dir_right(false);
		motors_set_speed_both(-1*left_speed, right_speed);
	} else if (left_speed > 0 && right_speed < 0 ){
		motors_set_dir_left(true);
		motors_set_dir_right(true);
		motors_set_speed_both(left_speed, -1*right_speed);
	} else{
		motors_set_dir_left(false);
		motors_set_dir_right(true);
		motors_set_speed_both(-1*left_speed, -1*right_speed);
	}
}

void motors_stop_all(void)
{
	motors_enable_all(false);

	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); /* stop Timer-1 */
	TCCR3B &= ~(_BV(CS32) | _BV(CS31) | _BV(CS30)); /* stop Timer-3 */
}



/* for future use */
/* — API to reset & read counts atomically — */
void motors_reset_edge_counts(void)
{
	uint8_t oldSREG = SREG;
	cli();
	left_edge_cnt = right_edge_cnt = 0;
	SREG = oldSREG;
}

uint32_t motors_get_edge_count_left(void)
{
	uint32_t c;
	uint8_t oldSREG = SREG;
	cli();
	c = left_edge_cnt;
	SREG = oldSREG;
	return c;
}

uint32_t motors_get_edge_count_right(void)
{
	uint32_t c;
	uint8_t oldSREG = SREG;
	cli();
	c = right_edge_cnt;
	SREG = oldSREG;
	return c;
}

uint32_t motors_get_step_count_left(void)
{
	return motors_get_edge_count_left() >> 1;
}

uint32_t motors_get_step_count_right(void)
{
	return motors_get_edge_count_right() >> 1;
}