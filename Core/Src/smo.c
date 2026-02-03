/*
 * smo->c
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#include "smo.h"
#include "foc_core.h"

extern MotorParams_t motor_params;

static float smo_sign(float val, float width) {
    if (val > width) return 1.0f;
    if (val < -width) return -1.0f;
    return val / width;
}

void SMO_Init(SMO_Handle_t* smo) {
    // k_gain: 전류 오차 보정 게인 (전압 단위, V_bus 근처 권장)
    smo->k_gain = 10.0f;  // 예: 10 ~ 50 사이

    // g: BEMF 관측기 게인 (반응 속도, 0.0 ~ 1.0 사이, 작을수록 필터링 강함)
    smo->g = 0.1f;       // 예: 0.05 ~ 0.5

    smo->omega_alpha_filter = 0.2f; // 속도 필터
    smo->min_operating_emf = 0.1f;   // 0.5V 이하는 정지로 간주
    //smo->phase_advance_gain = 0.0f;

    // 초기화
    smo->i_alpha_est = 0.0f;
    smo->i_beta_est = 0.0f;
    smo->e_alpha_est = 0.0f;
    smo->e_beta_est = 0.0f;
    smo->theta_est = 0.0f;

    smo->omega_est = 0.0f;
    smo->velocity_est = 0.0f;
    smo->last_i_alpha_error = 0.0f;
    smo->last_i_beta_error = 0.0f;
    smo->last_e_theta = 0.0f;
}

void SMO_Update_Arctan(SMO_Handle_t* smo, float v_alpha, float v_beta, float i_alpha, float i_beta) {
	if (isnan(smo->i_alpha_est) || isnan(smo->i_beta_est) || isnan(smo->e_alpha_est) || isnan(smo->e_beta_est) || isnan(smo->last_i_alpha_error)) {
		smo->i_alpha_est = 0.0f;
		smo->i_beta_est = 0.0f;
		smo->e_alpha_est = 0.0f;
		smo->e_beta_est = 0.0f;
		smo->last_i_alpha_error = 0.0f;
		smo->last_i_beta_error = 0.0f;
		smo->theta_est = 0.0f;
		smo->omega_est = 0.0f;
		smo->velocity_est = 0.0f;
		smo->last_e_theta = 0.0;
	}
	smo->v_alpha = v_alpha;
	smo->v_beta = v_beta;
	smo->i_alpha = i_alpha;
	smo->i_beta = i_beta;

	// A = 1 - (R * Ts) / L
	smo->A = 1.0f - (motor_params.R_motor * smo->Ts) / motor_params.L_motor;

	// b = Ts / L
	smo->b = smo->Ts / motor_params.L_motor;
	if (smo->b > 0.0000001f) {
		smo->b_inv = 1.0f / smo->b;
	} else {
		smo->b_inv = 0.0f;
	}

    // 1. Current Estimation Error
    float i_alpha_error = smo->i_alpha_est - i_alpha;
    float i_beta_error  = smo->i_beta_est  - i_beta;

    // 2. Sliding Control Signal (Z)
    // soft sign을 사용하여 채터링을 줄임
    float z_alpha = smo->k_gain * smo_sign(i_alpha_error, 0.1f);
    float z_beta  = smo->k_gain * smo_sign(i_beta_error, 0.1f);

    // 3. Current Observer Update
    // I_est = A * I_est + b * (V - E) - Z
    smo->i_alpha_est = smo->A * smo->i_alpha_est + smo->b * (v_alpha - smo->e_alpha_est) - z_alpha;
    smo->i_beta_est  = smo->A * smo->i_beta_est  + smo->b * (v_beta  - smo->e_beta_est)  - z_beta;

    // 4. Back-EMF Estimation (Disturbance Observer)
    // 이전 스텝의 Z값을 사용하여 역기전력을 적분
    float last_z_alpha = smo->k_gain * smo_sign(smo->last_i_alpha_error, 0.1f);
    float last_z_beta  = smo->k_gain * smo_sign(smo->last_i_beta_error,  0.1f);

    // E_new = E_old + Gain * (Error_dynamics)
    smo->e_alpha_est += smo->b_inv * smo->g * (i_alpha_error - smo->A * smo->last_i_alpha_error + last_z_alpha);
    smo->e_beta_est  += smo->b_inv * smo->g * (i_beta_error  - smo->A * smo->last_i_beta_error  + last_z_beta);

    // Error History Update
    smo->last_i_alpha_error = i_alpha_error;
    smo->last_i_beta_error  = i_beta_error;

    // 5. Angle & Speed Calculation
    smo->emf_magnitude = sqrtf(smo->e_alpha_est * smo->e_alpha_est + smo->e_beta_est * smo->e_beta_est);

    //if (smo->emf_magnitude >= smo->min_operating_emf) {
    // (1) BEMF Angle Calculation
    smo->e_theta = atan2f(smo->e_beta_est, smo->e_alpha_est);

    // (2) Phase Compensation (90 degree shift)
    if (smo->omega_est >= 0)
    	smo->theta_est = smo->e_theta - _PI_3 * 1.5f; // -90도 (-PI/2)
    else
    	smo->theta_est = smo->e_theta + _PI_3 * 1.5f; // +90도 (+PI/2)
    float phase_comp = smo->omega_est * smo->phase_advance_gain;
    smo->theta_est = smo->theta_est + phase_comp;

    // Angle Wrapping (0 ~ 2PI)
    if (smo->theta_est < 0.0f) smo->theta_est += _2PI;
    if (smo->theta_est > _2PI) smo->theta_est -= _2PI;

    // (3) Speed Estimation (Angle Differentiation)
    float delta_theta = smo->e_theta - smo->last_e_theta;
    // Wrap delta theta (-PI ~ PI)
    if (delta_theta > _PI)  delta_theta -= _2PI;
    if (delta_theta < -_PI) delta_theta += _2PI;

    // Calculate Omega (Mechanical Rad/s or Electrical Rad/s depending on usage)
    // 여기선 Electrical Rad/s로 계산
    float omega_instant = delta_theta / smo->Ts;

    // LPF for Speed
    smo->omega_est = smo->omega_est * (1.0f - smo->omega_alpha_filter) + omega_instant * smo->omega_alpha_filter;

    smo->last_e_theta = smo->e_theta;

    smo->velocity_est = smo->omega_est / (float)motor_params.pole_pairs;
    // } else {
    // 저속/정지 구간에서는 추정 중단
    // smo->omega_est = 0.0f; // 필요시 주석 해제
    //}
}

