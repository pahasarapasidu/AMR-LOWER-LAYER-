/*
 * profiler.h
 *
 * Created: 5/6/2025 4:34:35 AM
 *  Author: Endeavor360
 */ 

#ifndef PROFILER_H
#define PROFILER_H

/* Advance the profile generator by one tick.
 * Outputs: commanded linear velocity (cm/s) and angular velocity (deg/s).
 */
void profiler_step(float *vel_cm_s, float *omega_deg_s);

#endif /* PROFILER_H */
