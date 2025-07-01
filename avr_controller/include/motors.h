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

void motors_init(void);
void motors_enable_left(bool en);
void motors_enable_right(bool en);
void motors_enable_all(bool en);
void motors_set_dir_left(bool fwd);
void motors_set_dir_right(bool fwd);
void motors_set_speed_left(uint16_t rpm);
void motors_set_speed_right(uint16_t rpm);
void motors_set_speed_both(uint16_t rpm_left, uint16_t rpm_right);
void motors_move_left(int32_t steps);
void motors_move_right(int32_t steps);
void motors_stop_all(void);

/* � Step?count feedback (via OC?compare interrupts) � */
void motors_reset_edge_counts(void);
uint32_t motors_get_edge_count_left(void);
uint32_t motors_get_edge_count_right(void);
uint32_t motors_get_step_count_left(void);
uint32_t motors_get_step_count_right(void);

#endif // MOTORS_H