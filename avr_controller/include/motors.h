/*
 * motors.h
 *
 * Created: 5/6/2025 4:25:54 AM
 *  Author: Endeavor360
 */ 

#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

// Call once at startup
void motors_init(void);

// Enable/disable each motor (true = enabled)
void motors_enable_left (bool en);
void motors_enable_right(bool en);

// Set direction (true = forward/CW, false = reverse/CCW)
void motors_set_dir_left (bool forward);
void motors_set_dir_right(bool forward);

// Set speed in RPM (configures hardware timer)
void motors_set_speed_left (uint16_t rpm);
void motors_set_speed_right(uint16_t rpm);

// Blocking move: toggles PUL line in software for <steps> pulses
void motors_move_left (int32_t steps);
void motors_move_right(int32_t steps);

// Immediately stop and disable both motors
void motors_stop_all(void);

#endif // MOTORS_H