/*
 * foc_hw.c
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */
#include "foc_hw.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

uint16_t adc1_buf[4];
uint16_t adc2_buf[4];
uint16_t angle_raw = 0;
volatile uint8_t is_foc_enabled = 0;

#define CURRENT_SENSE_GAIN -0.01678f

void HW_FOC_Init(void) {
    HAL_OPAMP_SelfCalibrate(&hopamp1);
    HAL_OPAMP_SelfCalibrate(&hopamp2);
    HAL_OPAMP_SelfCalibrate(&hopamp3);

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
    HAL_Delay(10);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, 1);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, 1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

static int is_parity_ok(uint16_t data) {
    uint8_t parity = 0;
    for (int i = 0; i < 16; i++) {
        if (data & (1 << i)) parity++;
    }
    return (parity % 2) == 0;
}

void HW_Update_Encoder_BB(void) {
    uint16_t spi_raw_data = 0;
    CS_LOW();
    for (int i = 0; i < 16; ++i) {
        CLK_HIGH();
        for(volatile int k=0; k<1; ++k);
        spi_raw_data = (spi_raw_data << 1);
        if (MISO_READ()) spi_raw_data |= 1U;
        CLK_LOW();
        for(volatile int k=0; k<1; ++k);
    }
    CS_HIGH();

    if (spi_raw_data == 0x0000 || spi_raw_data == 0xFFFF) return;
    if (!is_parity_ok(spi_raw_data)) return;

    angle_raw = spi_raw_data & 0x3FFF;
}

void HW_Read_Currents(float* ia, float* ib, float* ic) {
    *ia = ((float)adc1_buf[0] - offset.ia_offset) * CURRENT_SENSE_GAIN;
    *ib = ((float)adc2_buf[0] - offset.ib_offset) * CURRENT_SENSE_GAIN;
    *ic = - foc_state.i_a - foc_state.i_b;
}

void HW_Write_PWM(float da, float db, float dc) {
    float Ts = (float)(htim1.Init.Period + 1);
    TIM1->CCR1 = (uint32_t)(da * Ts);
    TIM1->CCR2 = (uint32_t)(db * Ts);
    TIM1->CCR3 = (uint32_t)(dc * Ts);
}

void HW_Calibrate_Current_Offset(void) {
    long sum_adc1 = 0, sum_adc2 = 0;
    const int samples = 2048;

    TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
    HAL_Delay(200);

    for(int i=0; i<samples; ++i) {
        sum_adc1 += adc1_buf[0];
        sum_adc2 += adc2_buf[0];
        HAL_Delay(1);
    }
    offset.ia_offset = (float)sum_adc1 / samples;
    offset.ib_offset = (float)sum_adc2 / samples;
}

void HW_Align_Encoder(void) {
    float align_V = motor_params.align_voltage;

    for(int i=0; i<1000; ++i) {
        FOC_Calc_SVPWM(align_V, 0.0f); // Alpha=Valign, Beta=0
        HW_Write_PWM(foc_state.duty_a, foc_state.duty_b, foc_state.duty_c);
        HAL_Delay(1);
    }

    HW_Update_Encoder_BB();
    float cur_mech = angle_raw * ENC_TO_RAD;

    foc_state.theta_offset = cur_mech * (float)motor_params.pole_pairs;
    foc_state.theta_offset = fmodf(foc_state.theta_offset, _2PI);

    HW_Write_PWM(0.0f, 0.0f, 0.0f);
    HAL_Delay(500);
}


