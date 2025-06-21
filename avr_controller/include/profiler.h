/*
 * profiler.h
 *
 * Created: 5/21/2025 9:20:39 PM
 *  Author: Endeavor360
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#include <stdbool.h>

/** ?? LINEAR (translation) PROFILE ??????????????????????????????????????? */
/**
 * @brief  Initialise a 1D trapezoidal profile for straight?line motion.
 * @param  distance_mm   total travel distance [mm]
 * @param  max_vel_mm_s  peak velocity [mm/s]
 * @param  acc_mm_s2     accel/decel [mm/s�]
 */
void profiler_init(float distance_mm,
                   float max_vel_mm_s,
                   float acc_mm_s2);

/**
 * @brief  Must be called repeatedly in your main loop to update motor set-points.
 */
void profiler_update(void);

/**
 * @brief  Returns true while the linear profile is still running.
 */
bool profiler_is_running(void);

/** ?? ROTATION (in-place turn) PROFILE ??????????????????????????????????? */
/**
 * @brief  Initialise an in-place rotation trapezoid.
 * @param  angle_deg         target rotation, + = CCW, � = CW [�]
 * @param  max_omega_deg_s   peak angular speed [�/s]
 * @param  ang_acc_deg_s2    angular accel/decel [�/s�]
 */
void profiler_turn_init(float angle_deg,
                        float max_omega_deg_s,
                        float ang_acc_deg_s2);

/**
 * @brief  Must be called repeatedly in your main loop to update motor set-points.
 */
void profiler_turn_update(void);

/**
 * @brief  Returns true while the rotation profile is still running.
 */
bool profiler_turn_is_running(void);

#endif // PROFILER_H_