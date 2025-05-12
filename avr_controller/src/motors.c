/*
 * motors.c
 *
 * Created: 5/6/2025 4:26:05 AM
 * Author: Endeavor360
 */
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>
#include "motors.h"
#include "config.h"

/* private state ----------------------------------------------------------- */
static uint8_t left_top;   /* Timer-0 is 8-bit  */
static uint16_t right_top; /* Timer-1 is 16-bit */

/* helpers ----------------------------------------------------------------- */
static inline uint32_t rpm_to_freq(uint16_t rpm)
{
	/* steps per minute ? steps per second = rpm * STEPS_PER_REV / 60 */
	return ((uint32_t)rpm * STEPS_PER_REV) / 60U;
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

	/* ---------- Timer-0 (LEFT) � CTC, toggle OC0A, clock stopped -------- */
	TCCR0A = _BV(COM0A0) | _BV(WGM01); /* toggle on compare, CTC */
	TCCR0B = 0;						   /* prescaler 0 ? stopped  */
	OCR0A = 0;

	/* ---------- Timer-1 (RIGHT) � CTC, toggle OC1A, clock stopped ------- */
	TCCR1A = _BV(COM1A0); /* toggle on compare       */
	TCCR1B = _BV(WGM12);  /* CTC, prescaler 0        */
	OCR1A = 0;
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

void motors_set_speed_left(uint16_t rpm)
{
	uint32_t freq = rpm_to_freq(rpm);
	uint32_t top = (F_CPU / (2UL * freq * CLOCK_DIVISOR)) - 1UL;
	if (top > 255UL)
		top = 255UL; /* 8-bit limit */

	left_top = (uint8_t)top;
	OCR0A = left_top;

	/* start Timer-0 with /1024 prescale */
	TCCR0B = PRE_SCALE_TIMER0;
}

void motors_set_speed_right(uint16_t rpm)
{
	uint32_t freq = rpm_to_freq(rpm);
	uint32_t top = (F_CPU / (2UL * freq * CLOCK_DIVISOR)) - 1UL;

	right_top = (uint16_t)top;
	OCR1A = right_top;

	/* start Timer-1 with /1024 prescale */
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); /* clear first   */
	TCCR1B |= PRE_SCALE_TIMER1;
}

void motors_set_speed_both(uint16_t rpm_left, uint16_t rpm_right)
{
	motors_set_speed_left(rpm_left);
	motors_set_speed_right(rpm_right);
}

void motors_move_left(int32_t steps)
{
	motors_enable_left(true);
	if (steps < 0)
	{
		steps = -steps;
		motors_set_dir_left(false);
	}
	else
		motors_set_dir_left(true);

	for (int32_t i = 0; i < steps; ++i)
	{
		LEFT_PUL_PORT |= _BV(LEFT_PUL_BIT);
		_delay_us(5);
		LEFT_PUL_PORT &= ~_BV(LEFT_PUL_BIT);
		_delay_us(5);
	}
}

void motors_move_right(int32_t steps)
{
	motors_enable_right(true);
	if (steps < 0)
	{
		steps = -steps;
		motors_set_dir_right(false);
	}
	else
		motors_set_dir_right(true);

	for (int32_t i = 0; i < steps; ++i)
	{
		RIGHT_PUL_PORT |= _BV(RIGHT_PUL_BIT);
		_delay_us(5);
		RIGHT_PUL_PORT &= ~_BV(RIGHT_PUL_BIT);
		_delay_us(5);
	}
}

void motors_stop_all(void)
{
	/* disable drivers */
	motors_enable_all(false);

	/* stop timers � clear prescaler bits */
	TCCR0B &= ~(_BV(CS02) | _BV(CS01) | _BV(CS00));
	TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10));
}