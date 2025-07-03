/*
 * encoder.h
 *
 * Created: 5/6/2025 4:44:05 AM
 *  Author: Endeavor360
 */ 


#ifndef ENCODERS_H_
#define ENCODERS_H_


#include <stdint.h>
#include <stdbool.h> 
#include "systime.h"

/* ---------- API ---------- */
void     encoder_init(void);

int32_t  encoder_get_left(void);
int32_t  encoder_get_right(void);

void     encoder_reset_left(void);
void     encoder_reset_right(void);
void     encoder_reset_both(void);

bool encoder_emergency_hit(void);

/* ------------- high-level odometry helpers ------------- */
void      encoder_odometry_reset(void);
void      encoder_odometry_update(void);      /* call at ~1 kHz            */

float     encoder_left_speed_mm_s(void);
float     encoder_right_speed_mm_s(void);

float     encoder_robot_speed_mm_s(void);     /* forward speed             */
float     encoder_robot_omega_dps(void);      /* yaw rate (°/s)            */

float     encoder_robot_distance_mm(void);    /* travelled distance        */
float     encoder_robot_angle_deg(void);      /* accumulated heading       */

uint32_t  encoder_loop_time_us(void);         /* ?t used in last update    */

#endif /* ENCODERS_H_ */