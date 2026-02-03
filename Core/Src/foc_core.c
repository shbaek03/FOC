/*
 * foc_core.c
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#include "foc_core.h"

FOCState_t foc_state = {0};
CurrentOffset_t offset = {2048.0f, 2048.0f};

MotorParams_t motor_params = {
    .pole_pairs = 7,
    .max_voltage = 10.0f,
    .max_current = 10.0f,
    .align_voltage = 2.0f,
    .max_velocity = 800.0f,
    .R_motor = 0.072479795f,
    .L_motor = 0.0000217f,
	.bandwidth = 100.0f
};

PID_t pid_iq = {.Kp = 0.0f, .Ki = 0.0f, .limit = 10.0f};
PID_t pid_id = {.Kp = 0.0f, .Ki = 0.0f, .limit = 10.0f};
PID_t pid_vel = {.Kp = 0.01f, .Ki = 0.0001f, .limit = 10.0f};
PID_t pid_pos = {.Kp = 100.0f, .Ki = 0.0f, .Kd = 10.0f, .limit = 10.0f};

static float i_q_filtered = 0.0f;
static float i_d_filtered = 0.0f;
static float alpha_current = 0.1f;
static uint16_t vel_loop_cnt = 0;
static uint16_t pos_loop_cnt = 0;

static uint16_t pos_loop_devider = 20;
static uint16_t vel_loop_devider = 10;

#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void FOC_Core_Init(void) {
    pid_iq.Kp = motor_params.L_motor * motor_params.bandwidth * 2 * _PI;
    pid_iq.Ki = motor_params.R_motor * motor_params.bandwidth * 2 * _PI;
    pid_id.Kp = pid_iq.Kp;
    pid_id.Ki = pid_iq.Ki;

    foc_state.control_mode = MODE_CURRENT;
    foc_state.theta_offset = 0.0f;
}

void FOC_Update_Total_Angle(void) {
    float d_angle = foc_state.theta_mech - foc_state.prev_mech_angle;
    if (d_angle < -_PI) foc_state.rotation_count++;
    else if (d_angle > _PI) foc_state.rotation_count--;

    foc_state.total_angle = (float)foc_state.rotation_count * _2PI + foc_state.theta_mech;
    foc_state.prev_mech_angle = foc_state.theta_mech;
}

void FOC_Calc_SVPWM(float v_alpha, float v_beta) {
    float abs_V = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    float theta_V = atan2f(v_beta, v_alpha);

    if (theta_V < 0) theta_V += _2PI;

    int sector = (int)(theta_V / _PI_3) + 1;
    if (sector > 6) sector = 1;

    float theta_rel = theta_V - ((float)(sector - 1) * _PI_3);

    float max_V = motor_params.max_voltage * 0.57735f; // Vdc/sqrt(3)
    float M = abs_V / max_V;
    if (M > 1.0f) M = 1.0f;

    float t1 = M * sinf(_PI_3 - theta_rel);
    float t2 = M * sinf(theta_rel);
    float t0 = 1.0f - t1 - t2;
    if (t0 < 0.0f) t0 = 0.0f;

    float ta, tb, tc;

    switch(sector) {
        case 1: ta = t1+t2+0.5f*t0; tb = t2+0.5f*t0;    tc = 0.5f*t0; break;
        case 2: ta = t1+0.5f*t0;    tb = t1+t2+0.5f*t0; tc = 0.5f*t0; break;
        case 3: ta = 0.5f*t0;       tb = t1+t2+0.5f*t0; tc = t1+0.5f*t0; break;
        case 4: ta = 0.5f*t0;       tb = t1+0.5f*t0;    tc = t1+t2+0.5f*t0; break;
        case 5: ta = t2+0.5f*t0;    tb = 0.5f*t0;       tc = t1+t2+0.5f*t0; break;
        case 6: ta = t1+t2+0.5f*t0; tb = 0.5f*t0;       tc = t1+0.5f*t0; break;
        default: ta=0.5f; tb=0.5f; tc=0.5f; break;
    }
    foc_state.duty_a = ta;
    foc_state.duty_b = tb;
    foc_state.duty_c = tc;
}

void FOC_Cascade_Control_Loop(float dt) {
    float angle_diff = foc_state.total_angle - foc_state.prev_total_angle_vel;
    foc_state.prev_total_angle_vel = foc_state.total_angle;

    float raw_vel = angle_diff / dt;
    float alpha_vel = 0.02f;
    foc_state.velocity_filtered = foc_state.velocity_filtered * (1.0f - alpha_vel) + raw_vel * alpha_vel;
    foc_state.velocity = foc_state.velocity_filtered;

    // 1. Position Loop
    if (foc_state.control_mode == MODE_POSITION) {
        pos_loop_cnt++;
        if (pos_loop_cnt >= pos_loop_devider) {
            pos_loop_cnt = 0;
            float dt_pos = dt * pos_loop_devider;

            float vel_tgt = PID_Update(&pid_pos, foc_state.target_angle, foc_state.total_angle, dt_pos);
            foc_state.target_velocity = CONSTRAIN(vel_tgt, -motor_params.max_velocity, motor_params.max_velocity);

            if (fabsf(foc_state.target_angle - foc_state.total_angle) < 0.001f) {
                pid_vel.integral_err = 0.0f;
            }
        }
    }

    // 2. Velocity Loop
    if (foc_state.control_mode >= MODE_VELOCITY) {
        vel_loop_cnt++;
        if (vel_loop_cnt >= vel_loop_devider) {
            vel_loop_cnt = 0;
            float dt_vel = dt * vel_loop_devider;

            // Voltage Saturation Check
            float v_sq = foc_state.v_d * foc_state.v_d + foc_state.v_q * foc_state.v_q;
            float v_lim_sq = motor_params.max_voltage * motor_params.max_voltage;
            uint8_t is_sat = (v_sq >= v_lim_sq) ? 1 : 0;

            float iq_tgt = PID_Update_With_Sat(&pid_vel, foc_state.target_velocity, foc_state.velocity_filtered, dt_vel, is_sat);
            foc_state.target_Iq = CONSTRAIN(iq_tgt, -motor_params.max_current, motor_params.max_current);
        }
    }
}

void FOC_Current_Control_Loop(float dt) {
	float theta = foc_state.theta_elec;
	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);

	// Offset removal
	float mid = (foc_state.i_a + foc_state.i_b + foc_state.i_c) / 3.0f;
	foc_state.i_a -= mid;
	foc_state.i_b -= mid;
	foc_state.i_c -= mid;

	// 1. Clarke Transform (a,b,c -> alpha, beta)
	foc_state.i_alpha = foc_state.i_a;
	foc_state.i_beta = (1.0f / _SQRT3) * (foc_state.i_a + 2.0f * foc_state.i_b); // or (1 / _SQRT3) * (foc_state.i_b - foc_state.i_c)

	// 2. Park Transform (alpha, beta -> d, q)
	foc_state.i_d = foc_state.i_alpha * cos_theta + foc_state.i_beta * sin_theta;
	foc_state.i_q = - foc_state.i_alpha * sin_theta + foc_state.i_beta * cos_theta;

    // 4. Current Loop
    i_d_filtered = i_d_filtered * (1.0f - alpha_current) + foc_state.i_d * alpha_current;
    i_q_filtered = i_q_filtered * (1.0f - alpha_current) + foc_state.i_q * alpha_current;

    foc_state.v_d = PID_Update(&pid_id, foc_state.target_Id, i_d_filtered, dt);
    foc_state.v_q = PID_Update(&pid_iq, foc_state.target_Iq, i_q_filtered, dt);

    foc_state.i_d = i_d_filtered;
    foc_state.i_q = i_q_filtered;

    // 5. Inverse Park Transform
    foc_state.v_alpha = foc_state.v_d * cos_theta - foc_state.v_q * sin_theta;
    foc_state.v_beta  = foc_state.v_d * sin_theta + foc_state.v_q * cos_theta;

    // 6. Calc SVPWM
    FOC_Calc_SVPWM(foc_state.v_alpha, foc_state.v_beta);
}

void FOC_Change_Mode(ControlMode_t new_mode) {
    if (new_mode == foc_state.control_mode) return;

    if (new_mode == MODE_POSITION) {
        foc_state.target_angle = foc_state.total_angle;
    }
    else if (new_mode == MODE_VELOCITY) {
        foc_state.target_velocity = foc_state.velocity_filtered;
    }
    else {
        foc_state.target_Iq = 0.0f;
    }

    // PID Reset
    pid_pos.integral_err = 0.0f;
    pid_vel.integral_err = 0.0f;
    pid_pos.prev_err = 0.0f;

    foc_state.control_mode = new_mode;
}
