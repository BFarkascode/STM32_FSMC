/*
 *  Created on: Mar 10, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  Header file: ClockDriver_STM32F4xx.h
 *  Change history:
 */

#ifndef INC_CLOCKDRIVER_STM32L4X2_H_
#define INC_CLOCKDRIVER_STM32L4X2_H_

#include "stdint.h"
#include "stm32f412zx.h"

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE
extern uint8_t frame_end_flag;

//FUNCTION PROTOTYPES
void SysClockConfig(void);
void TIM6Config (void);
void Delay_us(int micro_sec);
void Delay_ms(int milli_sec);
void TIM3_CH2_PWM_Config (uint16_t PWM_resolution, uint16_t PWM_pulse);

#endif /* INC_CLOCKDRIVER_STM32L4X2_H_ */
