/*
 * foc_hw.h
 *
 *  Created on: 2026. 2. 3.
 *      Author: shbaek03
 */

#ifndef INC_FOC_HW_H_
#define INC_FOC_HW_H_

#include "main.h"
#include "foc_core.h"

// SPI Macros (BB)
#define CS_HIGH()   (GPIOB->BSRR = (1 << 8))
#define CS_LOW()    (GPIOB->BSRR = (1 << 24))
#define CLK_HIGH()  (GPIOB->BSRR = (1 << 6))
#define CLK_LOW()   (GPIOB->BSRR = (1 << 22))
#define MISO_READ() ((GPIOA->IDR & (1 << 15)) ? 1 : 0)

// External Buffers
extern uint16_t adc1_buf[4];
extern uint16_t adc2_buf[4];
extern uint16_t angle_raw;
extern volatile uint8_t is_foc_enabled;

// Functions
void HW_FOC_Init(void);      // Start ADC, PWM, OPAMP
void HW_Update_Encoder_BB(void);// SPI Bit-banging
void HW_Read_Currents(float* ia, float* ib, float* ic);
void HW_Write_PWM(float da, float db, float dc);
void HW_Calibrate_Current_Offset(void);
void HW_Align_Encoder(void);

#endif /* INC_FOC_HW_H_ */
