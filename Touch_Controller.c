/*
 *  Created on: Mar 15, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  File: Touch_Controller.c
 *  Change history:
 */

#include "Touch_Controller.h"

void TS_IO_Init(void){

	/*
	 * Toggle the CTP reset pin on PF12
	 */

	//CTP_reset
	RCC->AHB1ENR |=	(1<<5);														//PORTF clocking
	GPIOF->MODER |= (1<<24);													//GPIO output for PF12
	GPIOF->MODER &= ~(1<<25);													//GPIO output for PF12

	GPIOF->BSRR |= (1<<28);
	Delay_ms(5);
	GPIOF->BSRR |= (1<<12);
	Delay_ms(10);

}

void TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value){

	I2C1_Master_Start();
	I2C1_Address_TX(Addr);						//Note: Address is on [7:1] followed by the W/R bit. For W, it is 0.
	uint8_t TX_array[2] = {Reg, Value};
	I2C1_TX (2, &TX_array[0]);
	I2C1_Master_Stop();

}

uint8_t TS_IO_Read(uint8_t Addr, uint8_t Reg){

	/*
	 * I2C readout is sending over the register address as a write command
	 * followed by a read command on the slave address
	 *
	 */
	uint8_t data;
	I2C1_Master_Start();
	I2C1_Address_TX(Addr);						//we send the address as W
	uint8_t I2C_buf[1] = {Reg};
	I2C1_TX (1, &I2C_buf[0]);					//we send over the register to read out
	I2C1_Master_Stop();
	I2C1_Master_Start();
	I2C1_Address_RX(Addr, 1);					//we send over the address as R and the number of transfers
												//Note: Address is on [7:1] followed by the W/R bit. For R, it is 1.
	I2C1_RX (1, &I2C_buf[0]);					//we read out the singular reply into the buffer

	data = I2C_buf[0];
	return data;

}

uint16_t TS_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length){

	/*
	 * Multiple read function
	 *
	 */

	I2C1_Master_Start();
	I2C1_Address_TX(Addr);						//we send the address as W
	uint8_t I2C_buf[1] = {Reg};
	I2C1_TX (1, &I2C_buf[0]);					//we send over the register to read out
	I2C1_Master_Stop();
	I2C1_Master_Start();
	I2C1_Address_RX(Addr, Length);				//we send over the address as R and the number of transfers
												//Note: Address is on [7:1] followed by the W/R bit. For R, it is 1.
	I2C1_RX (Length, Buffer);					//we read out the singular reply into the buffer

	return;
}

void TS_IO_Delay(uint32_t Delay){

	Delay_ms(Delay);

}
