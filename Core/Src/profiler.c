/*
 * profiler.c
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#include "profiler.h"

float SCurve_Update(ProfileState_t *p, float target, float max_v, float max_a, float max_j, float dt) {
    if(dt <= 0.0f) return p->current_val;

    float dist_err = target - p->current_val;

    // 1. 목표 속도 계산 (SQRT 프로파일: 등가속도 운동 기반)
    // s = (v^2) / (2a) 공식을 역산하여 제동 거리를 고려한 속도 계산
    float v_target = sqrtf(2.0f * max_a * fabsf(dist_err));

    // 속도 제한 및 방향 설정
    if (v_target > max_v) v_target = max_v;
    if (dist_err < 0) v_target = -v_target;

    // 2. 필요한 가속도 계산 (P제어 형태)
    float v_err = v_target - p->current_vel;
    float a_needed = v_err / dt;

    // 3. 저크(Jerk) 제한 (가속도의 변화량 제한)
    float da = a_needed - p->current_accel;
    float max_da = max_j * dt;

    da = CONSTRAIN(da, -max_da, max_da);

    // 4. 상태 업데이트 (적분)
    p->current_accel += da;
    p->current_accel = CONSTRAIN(p->current_accel, -max_a, max_a);

    p->current_vel += p->current_accel * dt;
    p->current_vel = CONSTRAIN(p->current_vel, -max_v, max_v);

    p->current_val += p->current_vel * dt;

    return p->current_val;
}

