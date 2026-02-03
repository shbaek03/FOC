/*
 * smo.h
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#ifndef INC_SMO_H_
#define INC_SMO_H_

#include <math.h>

typedef struct {
    float v_alpha, v_beta;
    float i_alpha, i_beta;

    float A;        // 1 - R*Ts/L
    float b;        // Ts/L
    float b_inv;    // L/Ts
    float k_gain;   // Sliding Gain (K_slide)
    float g;        // BEMF Observer Gain
    float Ts;       // Sampling Time
    float min_operating_emf;

    // --- State Variables ---
    float i_alpha_est;
    float i_beta_est;

    float e_alpha_est; // Back-EMF (Estimated)
    float e_beta_est;

    float last_i_alpha_error;
    float last_i_beta_error;

    float emf_magnitude;
    float e_theta;      // BEMF Angle (Electrical)
    float theta_est;    // Estimated Rotor Angle
    float last_e_theta;

    float omega_est;    // Speed estimated from SMO
    float velocity_est; // mechanical anglular velocity (rad/s)
    float omega_alpha_filter; // Speed LPF Gain

    float phase_advance_gain;
} SMO_Handle_t;

void SMO_Init (SMO_Handle_t* smo);
void SMO_Update_Arctan (SMO_Handle_t* smo, float v_alpha, float v_beta, float i_alpha, float i_beta);

#endif /* INC_SMO_H_ */
