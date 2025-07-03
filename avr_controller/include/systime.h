/*
 * systime.h
 *
 * Created: 7/2/2025 12:05:12 PM
 *  Author: Endeavor360
 */ 


#ifndef SYSTIME_H_
#define SYSTIME_H_

#include <stdint.h>

/*  Call once at start-up */
void     systime_init(void);

/*  1-µs resolution, monotonic.
 *  Rolls over after ~292 000 years at 16 MHz (2?? half-µs)               */
uint64_t micros64(void);

#endif /* SYSTIME_H_ */
