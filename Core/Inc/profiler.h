/*
 * profiler.h
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#ifndef INC_PROFILER_H_
#define INC_PROFILER_H_

#include <stdint.h>
#include <math.h>

typedef struct {
	float current_val; //MODE_POSITION -> POS, MODE_VELOCITY -> VEL
	float current_vel; //MODE_POSITION -> VEL, MODE_VELOCITY -> ACCEL.
	float current_accel; ////MODE_POSITION -> ACCEL, MODE_VELOCITY -> JERK
	uint32_t last_time_us;
} ProfileState_t;

#define PROFILER_CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

float SCurve_Update(ProfileState_t *p, float target, float max_v, float max_a, float max_j, float dt);

#endif /* INC_PROFILER_H_ */
