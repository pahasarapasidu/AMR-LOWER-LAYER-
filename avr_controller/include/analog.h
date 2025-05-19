/*
 * analog.h – ADC sampler API (header?only stub)
 * -----------------------------------------
 * Placeholder prototypes for IR distance and battery monitor helpers.
 * Implement the definitions in adc.c later.
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#include "config.h"
#include <stdint.h>

/* initialise hardware (call once at start-up) */
void     analog_init(void);

/* blocking single-shot read of an arbitrary ADC channel */
uint16_t analog_read_raw(uint8_t channel);

/* helpers that deliver already-scaled values -------------*/
uint16_t analog_get_battery_1_mV(void);
uint16_t analog_get_battery_2_mV(void);

uint16_t analog_get_cliff_left(void);
uint16_t analog_get_cliff_front(void);
uint16_t analog_get_cliff_right(void);

#endif /* ANALOG_H_ */