/*
 *  Created on: Mar 15, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  Header file: I2CDriver_STM32F4xx.h
 *  Change history:
 */

#ifndef INC_I2CDRIVER_STM32L4X2_H_
#define INC_I2CDRIVER_STM32L4X2_H_

#include "stdint.h"
#include "stm32f412zx.h"

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE

//FUNCTION PROTOTYPES
void I2C1Config(void);
void I2C1_TX (uint8_t number_of_bytes, uint8_t *bytes_to_send);
void I2C1_Master_Start(void);
void I2C1_Address_TX(uint8_t slave_addr);
void I2C1_Address_RX(uint8_t slave_addr, uint8_t number_of_bytes);
void I2C1_RX (uint8_t number_of_bytes, uint8_t *bytes_received);
void I2C1_Master_Stop(void);


#endif /* INC_I2CDRIVER_STM32L4X2_H_ */
