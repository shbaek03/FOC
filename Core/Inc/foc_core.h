/*
 * foc_core.h
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#ifndef INC_FOC_CORE_H_
#define INC_FOC_CORE_H_

#include "pid.h"
#include <math.h>

// Constants
#define _PI         3.141592653f
#define _2PI        6.283185307f
#define _PI_3       1.04719755f
#define _SQRT3      1.73205081f
#define ENC_TO_RAD  (_2PI / 16384.0f)

// Types
typedef enum {
    MODE_CURRENT = 0,
    MODE_VELOCITY,
    MODE_POSITION
} ControlMode_t;

typedef struct {
    int pole_pairs;
    float L_motor;
    float R_motor;
    float max_voltage;
    float max_current;
    float max_velocity;
    float align_voltage;
    float bandwidth;
} MotorParams_t;

typedef struct {
    ControlMode_t control_mode;

    float target_angle;
    float target_velocity;
    float target_Iq;
    float target_Id;

    float total_angle;
    float prev_mech_angle;
    int32_t rotation_count;
    float prev_total_angle_vel;

    float velocity_filtered;
    float velocity; // Final velocity used

    float i_a, i_b, i_c;
    float i_alpha, i_beta;
    float i_d, i_q;

    float theta_mech;
    float theta_elec;
    float theta_offset;

    float v_d, v_q;
    float v_alpha, v_beta;
    float duty_a, duty_b, duty_c;
} FOCState_t;

typedef struct {
    float ia_offset;
    float ib_offset;
} CurrentOffset_t;

// Globals
extern FOCState_t foc_state;
extern MotorParams_t motor_params;
extern CurrentOffset_t offset;
extern PID_t pid_id, pid_iq, pid_vel, pid_pos;

// Functions
void FOC_Core_Init(void);
void FOC_Current_Control_Loop(float dt);
void FOC_Cascade_Control_Loop(float dt); //Velocity & Position Control Loop
void FOC_Update_Total_Angle(void);
void FOC_Change_Mode(ControlMode_t new_mode);
void FOC_Calc_SVPWM(float v_alpha, float v_beta);

#endif /* INC_FOC_CORE_H_ */
