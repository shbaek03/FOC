/*
 * pid.h
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>
#include <math.h>

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float integral_err;
	float prev_err;
	float limit;
} PID_t;

#define PID_Update(pid, target, meas, dt) \
    PID_Update_Full(pid, target, meas, dt, 0)
#define PID_Update_With_Sat(pid, target, meas, dt, sat) \
    PID_Update_Full(pid, target, meas, dt, sat)

float PID_Update_Full(PID_t *pid, float target, float measured, float dt, uint8_t is_saturated);


#endif /* INC_PID_H_ */
