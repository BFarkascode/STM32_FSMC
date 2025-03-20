/*
 *  Created on: Mar 15, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  Header file: Touch_Controller.h
 *  Change history:
 */

#ifndef INC_TOUCH_CONTROLLER_H_
#define INC_TOUCH_CONTROLLER_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "I2CDriver_STM32F4xx.h"

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE

//FUNCTION PROTOTYPES
void     TS_IO_Init(void);
void     TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t  TS_IO_Read(uint8_t Addr, uint8_t Reg);
uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
void     TS_IO_Delay(uint32_t Delay);

#endif /* INC_TOUCH_CONTROLLER_H_ */
