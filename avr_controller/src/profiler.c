/* -----------------------------------------------------------------------------
 * profiler.c Motion and trapezoidal profile manager
 *
 *
 * Author   : Endeavor360
 * Created  : 3rd July 2025
 * ---------------------------------------------------------------------------*/

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include <math.h>
#include "motors.h"
#include "encoder.h"
#include "profiler.h"

/* ====================  helpers =================== */
static inline float enc_loop_time_s(void)
{
	return (float)encoder_loop_time_us() * 1.0e-6f;
}

static inline float feedback_speed(ProfileKind k)
{
	return (k == PK_FORWARD) ? encoder_robot_speed_mm_s() : encoder_robot_omega_dps();
}

static float profile_braking_distance(const Profile *p)
{
	return fabsf(p->speed * p->speed - p->final_speed * p->final_speed) * 0.5f * p->one_over_acc;
}

/* ====================  Profile API =================== */
void profile_reset(Profile *p)
{
	cli();
	p->position = 0.0f;
	p->speed = 0.0f;
	p->target_speed = 0.0f;
	p->state = PS_IDLE;
	sei();
}

void profile_start(Profile *p, float distance, float top_speed, float final_speed, float acceleration)
{
	p->sign = (distance < 0.0f) ? -1 : +1;
	if (distance < 0.0f)
		distance = -distance;

	if (distance < 1.0f)
	{
		p->state = PS_FINISHED;
		return;
	}

	if (final_speed > top_speed)
		final_speed = top_speed;

	p->position = 0.0f;
	p->final_position = distance;

	p->target_speed = p->sign * fabsf(top_speed);
	p->final_speed = p->sign * fabsf(final_speed);

	p->acceleration = fabsf(acceleration);
	p->one_over_acc = (p->acceleration >= 1.0f) ? (1.0f / p->acceleration) : 1.0f;

	p->state = PS_ACCELERATING;
}

void profile_stop(Profile *p)
{
	cli();
	p->target_speed = 0.0f;
	p->speed = 0.0f;
	p->state = PS_FINISHED;
	sei();
}

void profile_update(Profile *p)
{
	if (p->state == PS_IDLE)
		return;

	float dt = enc_loop_time_s();
	float delta_v = p->acceleration * dt;
	float remaining = fabsf(p->final_position) - fabsf(p->position);

	if (p->state == PS_ACCELERATING)
	{
		if (remaining < profile_braking_distance(p))
		{
			p->state = PS_BRAKING;
			p->target_speed = (p->final_speed == 0.0f)? 0.0f: p->final_speed;
		}
	}

	/* reach target speed */
	if (p->speed < p->target_speed)
	{
		p->speed += delta_v;
		if (p->speed > p->target_speed)
			p->speed = p->target_speed;
	}
	else if (p->speed > p->target_speed)
	{
		p->speed -= delta_v;
		if (p->speed < p->target_speed)
			p->speed = p->target_speed;
	}

	/* integrate position with feedback */
	float fb_speed = feedback_speed(p->kind);
	p->position += fb_speed * dt;

	if (p->state != PS_FINISHED && remaining < 0.125f)
	{
		p->state = PS_FINISHED;
		p->target_speed = p->final_speed;
	}
}



/* ====================  Motion aggregate =================== */
MotionType motionType; /* single instance */


void motion_reset_drive_system(void)
{
	motors_stop_all();

	encoder_odometry_reset();
	profile_reset(&motionType.forward);
	profile_reset(&motionType.rotation);

	motors_enable_all(true);
}

void motion_stop(void) { motors_stop_all(); }

float motion_position(void) { return motionType.forward.position; }
float motion_velocity(void) { return motionType.forward.speed; }
float motion_acceleration(void) { return motionType.forward.acceleration; }

void motion_set_target_velocity(float v) { motionType.forward.target_speed = v; }

float motion_angle(void) { return motionType.rotation.position; }
float motion_omega(void) { return motionType.rotation.speed; }
float motion_alpha(void) { return motionType.rotation.acceleration; }

bool motion_move_finished(void)      { return motionType.forward.state == PS_FINISHED; }
bool motion_turn_finished(void) { return motionType.rotation.state == PS_FINISHED; }

void motion_start_move(float distance, float top_v, float final_v, float acc)
{
	motionType.forward.kind = PK_FORWARD;  // Add this line
	profile_start(&motionType.forward, distance, top_v, final_v, acc);
}

void motion_start_turn(float distance, float top_w, float final_w, float acc)
{
	motionType.rotation.kind = PK_ROTATION;  // Add this line
	profile_start(&motionType.rotation, distance, top_w, final_w, acc);
}

void motion_update(void)
{
	profile_update(&motionType.forward);
	profile_update(&motionType.rotation);
}


void motion_wait_until_position(float position_mm)
{
	while (motion_position() < position_mm)
	{
		_delay_ms(LOOP_TIME);
	}
}

void motion_wait_until_distance(float distance_mm)
{
	motion_wait_until_position(motion_position() + distance_mm);
}
