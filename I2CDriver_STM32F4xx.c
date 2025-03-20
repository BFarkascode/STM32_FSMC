/*
 *  Created on: Mar 15, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  File: I2CDriver_STM32F4xx.c
 *  Change history:
 */

/* Custom library to run I2C on F4xx
 * Be aware that the I2C peripheral of the F4xx is completely different than the L0xx
 * Unlike in the L0xx, we will have separate functions for generate Start/Stop conditions, send the Address and to Tx/Rx
 * Certain automatic capabilities like AUTOEND are not available in the F4xx
 * There are not separate registers for Tx/Rx/Address. All is sent from the same DR register
 * The Rx side has a complicated state machine where flags must be set according to where we are in the transmission
 * 		Failing to set the flags at the right time collapses the communication
 *
 * Note: the addressing is done with the << 1 shift assumed to be included (so, 0x38 address on the bus will be 0x70 for write, 0x71 for read)
 * */

#include "I2CDriver_STM32F4xx.h"


void I2C1Config(void) {

	/*
	 * 1) Enable clocking
	 * 2) GPIO setup
	 * 3) Set clock source - this is not possible on the F4xx. Unlike the L0xx, here all I2Care clocked from APB only!
	 * 4) Set timing
	 * 5) Set own address, if any
	 * 6)Set filtering and clock stretch
	 * 7)Enable I2C
	 *
	 */

	//1)Enable clocking in the RCC, set I2CCLK source clock, enable GPIO clocking - PB8 SCL, PB9 SDA

	RCC->APB1ENR |= (1<<21);							//enable I2C1 clock
	RCC->AHB1ENR |=	(1<<1);								//PORTB clocking

	//2)Set GPIO parameters (mode, speed, pullup) - PB6 SCL, PB7 SDA
	GPIOB->MODER &= ~(1<<12);							//alternate function for PB6
	GPIOB->MODER |= (1<<13);							//alternate function for PB6
	GPIOB->MODER &= ~(1<<14);							//alternate function for PB7
	GPIOB->MODER |= (1<<15);							//alternate function for PB6
														//Note: MODER resets to 0x0 here!
	GPIOB->OTYPER |= (1<<6);							//open drain for PB6
	GPIOB->OTYPER |= (1<<7);							//open drain for PB7
	GPIOB->OSPEEDR |= (3<<12);							//high speed PB6
	GPIOB->OSPEEDR |= (3<<14);							//high speed PB7
	GPIOB->PUPDR |= (1<<12);							//pullup PB6
	GPIOB->PUPDR |= (1<<14);							//pullup PB7

	//Note: AFR values are in the device datasheet
	GPIOB->AFR[0] |= (4<<24);							//PB6 AF4 setup
	GPIOB->AFR[0] |= (4<<28);							//PB7 AF4 setup

	Delay_us(1);										//this delay is necessary, otherwise GPIO setup may freeze

	//reset I2C
	I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);

	//3)Set a clock source for the internal clock of the I2C

	//N/A

	//4)Set timing - standard mode selected with 8 MHz I2CCLK

	//Standard timing
	I2C1->CR2 = 0x32;									//frequency of the APB1 clock. Currently 50 MHz (0x32 in hex).
	I2C1->CCR = 0x802a;									//value taken from CubeMX - CCR is clock control register
	I2C1->TRISE = 0x10;									//value taken from CubeMX

	//5) Set own address

	//N/A

	//6)Set filtering and clock stretch

	I2C1->FLTR &= ~(1<<4);								//analog filter enabled
	I2C1->CR1 &= ~(1<<7);								//clock stretch enabled
														//this must be kept as such for MASTER mode
	//7)Enable I2C
	I2C1->CR1 |= (1<<0);								//enable I2C
	Delay_us(1);										//We wait for the setup to take effect
}



void I2C1_Master_Start(void){
	/*
	 * 1) We generate a start bit
	 * 2) We enable the ACK
	 * 3)We wait for the SB bit to go HIGH indicating the the start condition is generated
	 *
	 *
	 * Note: for exact sequence, check the "I2C Start condition" and whatever comes afterwards
	 *
	 */

	//0)
	while((I2C1->SR2 & (1<<1)) == (1<<1));				//we wait while the bus is BSY

	//1)
	I2C1->CR1 |= (1<<10);								//ACK enabled

	//2)
	I2C1->CR1 |= (1<<8);								//we start
														//S event ends here

	//3)
	while((I2C1->SR1 & (1<<0)) == 0);					//we wait for the SB bit to be set

	uint32_t temp = I2C1->SR1;							//we clear SR1
														//Note: this MUST be done to progress the state machine

}


void I2C1_Address_TX(uint8_t slave_addr){

	/*
	 * 1) We load the slave address into the Tx/Rx/DR register
	 * 2) We wait until the ADDR bit is set, indicating that the address has been sent
	 * 3) We reset the SR registers
	 *
	 * Note: the ADDR bit ONLY goes HIGH, if there is a match in the address. As such, this function can only be used with known addresses
	 *
	 */

	//1)
	I2C1->DR = slave_addr;								//Note: this is adjusted to match the IO function a layer above
														//writing is LSB HIGH
														//EV5 event ends here

	//2)
	while((I2C1->SR1 & (1<<1)) == 0);					//wait for the ADDR bit to set
														//set only if the ADDR is ACKed by the slave
	//3)
	uint32_t temp = I2C1->SR1 | I2C1->SR2;				//reset SR registers and ADDR bit
														//Note: this MUST be done, see state machine for the I2C peripheral in F4xx
														//EV6 event ends here

}


void I2C1_TX (uint8_t number_of_bytes, uint8_t *bytes_to_send) {
	/*
	 * A set of while conditions to progress through a simple TX.
	 * Only the TXE and the BTF flags need to be checked
	 * Difference between TX and RX is done on the addressing level, not here. Here we are just sending stuff out.
	 *
	 */

	//EV8_1 followed by EV8 followed by EV8_2

	while((I2C1->SR1 & (1<<7)) == 0);						//TXE flag is LOW
	while(number_of_bytes) {

		while((I2C1->SR1 & (1<<7)) == 0);
		I2C1->DR = (volatile uint8_t) *bytes_to_send++;		//write data into DR
		number_of_bytes--;

	}

	while((I2C1->SR1 & (1<<7)) == 0);						//TXE flag is LOW

}


void I2C1_Address_RX(uint8_t slave_addr, uint8_t number_of_bytes){

	/*
	 * 1) We load the slave address into the Tx/Rx/DR register
	 * 2) We wait until the ADDR bit is set, indicating that the address has been sent
	 * 3) We reset the SR registers
	 *
	 * Note: the ADDR bit ONLY goes HIGH, if there is a match in the address. As such, this function can only be used with known addresses
	 *
	 */

	uint32_t temp;

	//1)
	I2C1->DR = (slave_addr + 1);						//Note: this is adjusted to match the IO function a layer above
														//reading is LSB HIGH
														//EV5 ends here

	//2)
	while((I2C1->SR1 & (1<<1)) == 0);					//wait for the ADDR bit to set
														//set only if the ADDR is ACKed by the slave

	//3) Configure the I2C peripheral according to message size
	//this below is to set the I2C to generate the proper signals
	//Note: this is just cloning the HAL_I2C_Mem_Read function
	if(number_of_bytes == 0){

		temp = I2C1->SR1 | I2C1->SR2;									//reset ADDR

		I2C1->CR1 |= (1<<9);											//generate STOP

	} else if (number_of_bytes == 1){

		I2C1->CR1 &= ~(1<<10);											//clear ACK

		temp = I2C1->SR1 | I2C1->SR2;									//reset ADDR

		I2C1->CR1 |= (1<<9);											//generate STOP

	} else if (number_of_bytes == 2){

		I2C1->CR1 &= ~(1<<10);											//clear ACK

		I2C1->CR1 |= (1<<11);											//set POS

	    temp = I2C1->SR1 | I2C1->SR2;									//reset ADDR

	} else {

		temp = I2C1->SR1 | I2C1->SR2;									//reset ADDR

	}

}


void I2C1_RX (uint8_t number_of_bytes, uint8_t *bytes_received) {
	/*
	 * A set of while conditions to progress through a simple RX.
	 * Only the TXE and the BTF flags need to be checked
	 * Difference between TX and RX is done on the addressing level, not here. Here we are just receiving stuff.
	 * BTF is NOT set after NACK, so we can't use it on the RX side
	 *
	 */

	uint32_t temp;

	while(number_of_bytes) {

		if(number_of_bytes <= 3){

			if (number_of_bytes == 1){

					while((I2C1->SR1 & (1<<6)) == 0);								//RXNE set

					*bytes_received++ = I2C1->DR;

					number_of_bytes--;

				} else if (number_of_bytes == 2){

					while((I2C1->SR1 & (1<<2)) == 0);								//BTF set

					I2C1->CR1 |= (1<<9);											//generate STOP

					*bytes_received++ = I2C1->DR;

					number_of_bytes--;

					*bytes_received++ = I2C1->DR;

					number_of_bytes--;

				} else {

					while((I2C1->SR1 & (1<<2)) == 0);								//BTF set

					I2C1->CR1 &= ~(1<<10);											//clear ACK

					*bytes_received++ = I2C1->DR;

					number_of_bytes--;

					while((I2C1->SR1 & (1<<2)) == 0);								//BTF set

					I2C1->CR1 |= (1<<9);											//generate STOP

					*bytes_received++ = I2C1->DR;

					number_of_bytes--;

					*bytes_received++ = I2C1->DR;

					number_of_bytes--;

				}

		} else {

			while((I2C1->SR1 & (1<<6)) == 0);
			*bytes_received++ = I2C1->DR;

			number_of_bytes--;


			if((I2C1->SR1 & (1<<2)) == (1<<2)){						//BTF flag)

				*bytes_received++ = I2C1->DR;

				number_of_bytes--;

			}

		}

	}

}

void I2C1_Master_Stop(void){

	/*
	 * Simple stop command to generate a stop condition on the bus
	 *
	 */

	I2C1->CR1 |= (1<<9);

}
