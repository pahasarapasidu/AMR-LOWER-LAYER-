/*
 * profiler.h
 *
 * Created: 5/21/2025 9:20:39 PM
 *  Author: Endeavor360
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#include <stdbool.h>
#include <stdint.h>

/* ====================  Profile types =================== */

typedef enum {
	PS_IDLE = 0,
	PS_ACCELERATING,
	PS_BRAKING,
	PS_FINISHED,
} ProfileState;

typedef enum {
	PK_FORWARD = 0,
	PK_ROTATION = 1,
} ProfileKind;

typedef struct {
	volatile ProfileState state;
	volatile float        speed;          /* mm/s or deg/s */
	volatile float        position;       /* mm   or deg   */

	int8_t        sign;          /* direction */
	ProfileKind   kind;          /* forward / rotation */

	float acceleration;
	float one_over_acc;

	float target_speed;
	float final_speed;
	float final_position;
} Profile;

/* Forward declaration of Motion aggregate */
typedef struct {
	Profile forward;
	Profile rotation;
} MotionType;

/* ====================  Profile API =================== */
void profile_reset(Profile *p);
void profile_start(Profile *p, float distance, float top_s, float final_s, float acc);
void profile_stop(Profile *p);
void profile_update(Profile *p);

/* ====================  Motion facade =================== */
void  motion_reset_drive_system(void);
void  motion_stop(void);
float motion_position(void);
float motion_velocity(void);
float motion_acceleration(void);
void  motion_set_target_velocity(float v);
float motion_angle(void);
float motion_omega(void);
float motion_alpha(void);
void  motion_start_move(float dist, float top_v, float final_v, float acc);
bool  motion_move_finished(void);
void  motion_start_turn(float dist, float top_w, float final_w, float acc);
bool  motion_turn_finished(void);
void  motion_update(void);
void  motion_wait_until_position(float pos_mm);
void  motion_wait_until_distance(float dist_mm);

#endif // PROFILER_H_