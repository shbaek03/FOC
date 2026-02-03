/*
 * pid.c
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */
#include "pid.h"

float PID_Update_Full(PID_t *pid, float target, float measured, float dt, uint8_t is_saturated) {
    float error = target - measured;
    float p_term = pid->Kp * error;

    if (is_saturated == 0) {
    	pid->integral_err += error * pid->Ki * dt;
    }
    else {
    	if ((error > 0 && pid->integral_err < 0) || (error < 0 && pid->integral_err > 0)) {
    		pid->integral_err += error * pid->Ki * dt;
    	}
    }

    if (pid->integral_err > pid->limit) pid->integral_err = pid->limit;
    else if (pid->integral_err < -pid->limit) pid->integral_err = -pid->limit;

    float d_term = 0.0f;
    if (dt > 0.0f) {
    	d_term = (error - pid->prev_err) / dt * pid->Kd;
    	pid->prev_err = error;
    }

    return p_term + pid->integral_err + d_term;
}

