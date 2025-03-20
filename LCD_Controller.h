/*
 *  Created on: Mar 10, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  Header file: LCD_Controller.h
 *  Change history:
 */

#ifndef INC_LCD_CONTROLLER_H_
#define INC_LCD_CONTROLLER_H_

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "stm32f412zx.h"


#define FMSC_BANK1_REG 		((uint16_t *) 0x60000000)						//register address for A0
#define FMSC_BANK1_DATA 	((uint16_t *) 0x60000002)						//data address for A0 -> A0<<1 but it also has an extra shift to HADDR[25:1] for 16 bits
																			//A0 is external memory address A[0]
																			//bit0 in the memory bank is not checked whatsoever - value is X/don't care
																			//A0 will be bit 2 in the actual comms due to the shift
																			//sending command, A0 is 0, i.e. we will be looking at memory address 0x60000000 (last bit is X, one before is 0)
																			//sending data, A0 is 1, i.e. we will be looking at memory address 0x60000002 (last bit is X, one before is 1)
																			//we have 0x6 at the start since that is what selects the FMSC bank - here, 0x6 is NOR, bank 1
																			//in the end, the 32 bits sequence is BANK select [31:28], NE1 select [27:26], stuff [25:1] and don't care [0]
																			//this only applies to 16 bits! At 8-bit, we don't have the bit[0] don't care!
																			//A0-A4 select will define the data offset (A4 will start data at 0x60000010)
																			//register select pretty much defines the size of the command we will be sending to the attached memory device
																			//A0 will be 16-bit commands, A4 will be 64-bit commands, for instance

//LOCAL CONSTANT

//LOCAL VARIABLE

//EXTERNAL VARIABLE
extern uint16_t image[57600];

//FUNCTION PROTOTYPES
void	 FSMC_LCD_Init(void);
void     LCD_IO_Init(void);
void     LCD_IO_WriteMultipleData(uint16_t *pData, uint32_t Size);
void     LCD_IO_WriteReg(uint8_t Reg);
void     LCD_IO_WriteData(uint16_t RegValue);
uint16_t LCD_IO_ReadData(void);
void     LCD_IO_Delay(uint32_t delay);
void 	 GenerateImage(void);
void 	 WipeImage(void);

#endif /* INC_LCD_CONTROLLER_H_ */
