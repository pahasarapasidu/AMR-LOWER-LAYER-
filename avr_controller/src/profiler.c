/*
 * profiler.c
 *
 * Created: 5/21/2025 9:21:21 PM
 *  Author: Endeavor360
 */

#include <math.h>
#include "profiler.h"
#include "motors.h"
#include "config.h"

/*---------------------------- LINEAR PROFILE STATE ----------------------------*/
static float target_dist_mm;
static float lin_max_vel;
static float lin_acc;
static float lin_counts_per_mm;
static uint32_t lin_start_steps;
static bool lin_running;

/*---------------------------- ROTATION PROFILE STATE----------------------------*/
static float target_wheel_mm;
static float turn_max_vel;
static float turn_acc;
static float turn_counts_per_mm;
static uint32_t turn_start_left, turn_start_right;
static bool turn_running;
static bool turn_ccw;

/*---------------------------- LINEAR PROFILE API ----------------------------*/
void profiler_init(float distance_mm,
				   float max_vel_mm_s,
				   float acc_mm_s2)
{
	target_dist_mm = distance_mm;
	lin_max_vel = max_vel_mm_s;
	lin_acc = acc_mm_s2;

	/* compute encoder counts per mm */
	float circ = WHEEL_DIAMETER_MM * M_PI;
	lin_counts_per_mm = (4.0f * (float)ENCODER_PPR) / circ;

	/* reset & snapshot encoder */
	motors_reset_edge_counts();
	lin_start_steps = motors_get_step_count_left();

	lin_running = true;
}

void profiler_update(void)
{
	if (!lin_running)
		return;

	/* distance so far [mm] */
	uint32_t steps = motors_get_step_count_left() - lin_start_steps;
	float dist_mm = steps / lin_counts_per_mm;

	/* trapezoid breakpoints */
	float d_acc = (lin_max_vel * lin_max_vel) / (2.0f * lin_acc);
	float d_decel = d_acc;
	float cruise_end = target_dist_mm - d_decel;

	/* compute desired v [mm/s] */
	float v;
	if (dist_mm < d_acc)
	{
		v = sqrtf(2.0f * lin_acc * dist_mm);
	}
	else if (dist_mm < cruise_end)
	{
		v = lin_max_vel;
	}
	else
	{
		float rem = target_dist_mm - dist_mm;
		v = sqrtf(2.0f * lin_acc * rem);
	}

	/* convert to RPM */
	float revs_s = v / (WHEEL_DIAMETER_MM * M_PI);
	uint16_t rpm = (uint16_t)(revs_s * 60.0f + 0.5f);

	/* command both wheels forward */
	motors_set_dir_left(true);
	motors_set_dir_right(true);
	motors_set_speed_both(rpm, rpm);

	/* done? */
	if (dist_mm >= target_dist_mm)
	{
		motors_stop_all();
		lin_running = false;
	}
}

bool profiler_is_running(void)
{
	return lin_running;
}

/*---------------------------- ROTATION PROFILE API -------------------------------*/
void profiler_turn_init(float angle_deg,
						float max_omega_deg_s,
						float ang_acc_deg_s2)
{
	/* 1) Compute how far each wheel must travel: d = (L/2)*?_rad */
	float theta = angle_deg * (M_PI / 180.0f);
	float half_track = WHEEL_BASE_MM * 0.5f;
	target_wheel_mm = fabsf(half_track * theta);
	turn_ccw = (angle_deg > 0.0f);

	/* 2) Convert angular ? linear at wheel rim */
	turn_max_vel = fabsf(max_omega_deg_s * (M_PI / 180.0f) * half_track);
	turn_acc = fabsf(ang_acc_deg_s2 * (M_PI / 180.0f) * half_track);

	/* 3) Encoder counts per mm (same formula) */
	float circ = WHEEL_DIAMETER_MM * M_PI;
	turn_counts_per_mm = (4.0f * (float)ENCODER_PPR) / circ;

	/* 4) Reset & snapshot both encoders */
	motors_reset_edge_counts();
	turn_start_left = motors_get_step_count_left();
	turn_start_right = motors_get_step_count_right();

	turn_running = true;
}

void profiler_turn_update(void)
{
	if (!turn_running)
		return;

	/* average wheel travel [mm] */
	uint32_t sl = motors_get_step_count_left() - turn_start_left;
	uint32_t sr = motors_get_step_count_right() - turn_start_right;
	float dl = sl / turn_counts_per_mm;
	float dr = sr / turn_counts_per_mm;
	float d = (dl + dr) * 0.5f;

	/* trapezoid breakpoints */
	float d_acc = (turn_max_vel * turn_max_vel) / (2.0f * turn_acc);
	float d_decel = d_acc;

	/* desired v [mm/s] */
	float v;
	if (d < d_acc)
	{
		v = sqrtf(2.0f * turn_acc * d);
	}
	else if (d < (target_wheel_mm - d_decel))
	{
		v = turn_max_vel;
	}
	else
	{
		float rem = target_wheel_mm - d;
		v = sqrtf(2.0f * turn_acc * rem);
	}

	/* convert to RPM */
	float revs_s = v / (WHEEL_DIAMETER_MM * M_PI);
	uint16_t rpm = (uint16_t)(revs_s * 60.0f + 0.5f);

	/* opposite wheel dirs for in-place turn */
	motors_set_dir_left(!turn_ccw);
	motors_set_dir_right(turn_ccw);
	motors_set_speed_both(rpm, rpm);

	/* done? */
	if (d >= target_wheel_mm)
	{
		motors_stop_all();
		turn_running = false;
	}
}

bool profiler_turn_is_running(void)
{
	return turn_running;
}
