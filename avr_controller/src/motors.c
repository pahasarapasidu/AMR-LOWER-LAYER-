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
	
	motors_enable_all(true);
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
	TCNT3 = 0;
	TCCR3B |= PRE_SCALE_TIMER3;
}

void motors_set_speed_right(uint16_t vel)
{
	uint32_t freq = velocity_to_freq(vel);
	uint32_t top = (F_CPU / (2UL * freq * CLOCK_DIVISOR_TIMER1)) - 1UL;

	right_top = (uint16_t)top;
	OCR1A = right_top;

	/* start Timer-1 with /1024 pre-scale */
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); /* clear first   */
	TCNT1 = 0;
	TCCR1B |= PRE_SCALE_TIMER1;
}

void motors_set_speed_both(uint16_t vel_left, uint16_t vel_right)
{
	motors_set_speed_left(vel_left);
	motors_set_speed_right(vel_right);
}

void motors_update( float velocity, float omega)
{
	/* Feed-forward terms based on desired wheel tangential speed */
	float tangent_speed = omega * WHEEL_BASE_MM * M_PI/ 360.0 ;
	float left_speed    = velocity - tangent_speed;
	float right_speed   = velocity + tangent_speed;

	// Get absolute speeds for motor control
	float left_abs = fabsf(left_speed);
	float right_abs = fabsf(right_speed);
	
	// Set directions based on speed signs
	motors_set_dir_left(left_speed >= 0);
	motors_set_dir_right(right_speed < 0); //invert due opposite orientation
	
	motors_set_speed_both((uint16_t)left_abs, (uint16_t)right_abs);
}

void motors_stop_all()
{
	motors_enable_all(false);

	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); /* stop Timer-1 */
	TCCR3B &= ~(_BV(CS32) | _BV(CS31) | _BV(CS30)); /* stop Timer-3 */
}