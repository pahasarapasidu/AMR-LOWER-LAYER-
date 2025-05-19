/*
 * systick.h – Timer?driven scheduler interface
 * Author : Kiran Gunathilaka
 * Date   : 13 May 2025
 */

#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/** Initialize Timer?3 and start the 200 Hz control ISR. */
void systick_init(void);

/**
 * Latest command set?points produced by the profiler, in physical units.
 * The main loop should read these once per cycle and update the motor
 * driver accordingly.
 */
extern volatile float g_targetVel;   /* cm s?¹ */
extern volatile float g_targetOmega; /* deg s?¹ */



#endif /* SYSTICK_H */
