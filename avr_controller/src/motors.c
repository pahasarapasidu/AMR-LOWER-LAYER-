/*
 * motors.c
 *
 * Created: 5/6/2025 4:26:05 AM
 *  Author: Endeavor360
 */ 
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#include "motors.h"
#include "config.h"

// “top” values for CTC mode
static uint16_t left_top;
static uint16_t right_top;

void motors_init(void) {
	// — configure GPIOs —
	LEFT_PUL_DDR  |= _BV(LEFT_PUL_BIT);
	LEFT_DIR_DDR  |= _BV(LEFT_DIR_BIT);
	LEFT_ENA_DDR  |= _BV(LEFT_ENA_BIT);

	RIGHT_PUL_DDR |= _BV(RIGHT_PUL_BIT);
	RIGHT_DIR_DDR |= _BV(RIGHT_DIR_BIT);
	RIGHT_ENA_DDR |= _BV(RIGHT_ENA_BIT);

	// defaults: pulses low, dirs low, disabled (ENA high)
	LEFT_PUL_PORT  &= ~_BV(LEFT_PUL_BIT);
	LEFT_DIR_PORT  &= ~_BV(LEFT_DIR_BIT);
	LEFT_ENA_PORT  |=  _BV(LEFT_ENA_BIT);

	RIGHT_PUL_PORT &= ~_BV(RIGHT_PUL_BIT);
	RIGHT_DIR_PORT &= ~_BV(RIGHT_DIR_BIT);
	RIGHT_ENA_PORT |=  _BV(RIGHT_ENA_BIT);

	// — Timer1 (16-bit) for LEFT motor PUL on OC1A (PB5) —
	TCCR1A = _BV(COM1A0);      // toggle OC1A on compare
	TCCR1B = _BV(WGM12);       // CTC mode, no clock yet

	// — Timer4 (16-bit) for RIGHT motor PUL on OC4D (PD7) —
	TCCR4A = _BV(COM4D0);      // toggle OC4D on compare
	TCCR4B = _BV(WGM41);       // CTC mode, no clock yet
}

void motors_enable_left(bool en) {
	if (en) LEFT_ENA_PORT &= ~_BV(LEFT_ENA_BIT);
	else    LEFT_ENA_PORT |=  _BV(LEFT_ENA_BIT);
}

void motors_enable_right(bool en) {
	if (en) RIGHT_ENA_PORT &= ~_BV(RIGHT_ENA_BIT);
	else    RIGHT_ENA_PORT |=  _BV(RIGHT_ENA_BIT);
}

void motors_set_dir_left(bool fwd) {
	if (fwd) LEFT_DIR_PORT |=  _BV(LEFT_DIR_BIT);
	else     LEFT_DIR_PORT &= ~_BV(LEFT_DIR_BIT);
}

void motors_set_dir_right(bool fwd) {
	if (fwd) RIGHT_DIR_PORT |=  _BV(RIGHT_DIR_BIT);
	else     RIGHT_DIR_PORT &= ~_BV(RIGHT_DIR_BIT);
}

void motors_set_speed_left(uint16_t rpm) {
	// freq = (rpm * steps/rev) / 60 ? top = F_CPU/(2·freq) – 1
	uint32_t freq = (uint32_t)rpm * STEPS_PER_REV / 60U;
	left_top = (uint16_t)(F_CPU/(2UL*freq) - 1UL);
	OCR1A    = left_top;
	TCCR1B  |= _BV(CS10);      // start timer1 with clk/1
}

void motors_set_speed_right(uint16_t rpm) {
	uint32_t freq = (uint32_t)rpm * STEPS_PER_REV / 60U;
	right_top = (uint16_t)(F_CPU/(2UL*freq) - 1UL);
	OCR4D     = right_top;
	TCCR4B   |= _BV(CS40);     // start timer4 with clk/1
}

void motors_move_left(int32_t steps) {
	motors_enable_left(true);
	if (steps < 0) { steps = -steps; motors_set_dir_left(false); }
	else            motors_set_dir_left(true);

	for (int32_t i = 0; i < steps; i++) {
		LEFT_PUL_PORT |=  _BV(LEFT_PUL_BIT);
		_delay_us(5);
		LEFT_PUL_PORT &= ~_BV(LEFT_PUL_BIT);
		_delay_us(5);
	}
}

void motors_move_right(int32_t steps) {
	motors_enable_right(true);
	if (steps < 0) { steps = -steps; motors_set_dir_right(false); }
	else            motors_set_dir_right(true);

	for (int32_t i = 0; i < steps; i++) {
		RIGHT_PUL_PORT |=  _BV(RIGHT_PUL_BIT);
		_delay_us(5);
		RIGHT_PUL_PORT &= ~_BV(RIGHT_PUL_BIT);
		_delay_us(5);
	}
}

void motors_stop_all(void) {
	// disable outputs
	motors_enable_left (false);
	motors_enable_right(false);
	// stop timers
	TCCR1B &= ~(_BV(CS12)|_BV(CS11)|_BV(CS10));
	TCCR4B &= ~(_BV(CS42)|_BV(CS41)|_BV(CS40));
}
