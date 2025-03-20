
/*
 *  Created on: Mar 10, 2025
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Program version: 1.0
 *  File: LCD_Controller.c
 *  Change history:
 */

#include "LCD_Controller.h"

void FSMC_LCD_Init(void){

	/*
	 * Calibration of the FSMC to run the LCD screen
	 * On the F412, the memory controller only works with NOR/PSRAM. (On others such as the F429, there are NAND and PCCard controllers as well!)
	 * Base bank address is at 0x60000000
	 * Bank size is 64MB (and we have 4 banks)
	 * FMSC runs on HCLK (here set as 100 MHz)
	 * Note: Changing HCLK will force a change on the timing setup!
	 *
	 */

	//Set clocking
	RCC->AHB1ENR |=	(1<<3);														//PORTD clocking
	RCC->AHB1ENR |=	(1<<4);														//PORTE clocking
	RCC->AHB1ENR |=	(1<<5);														//PORTF clocking

	//Set FSMC GPIOs alternate function
	GPIOD->MODER |= (1<<1);														//GPIO alternative for PD0
	GPIOD->MODER &= ~(1<<0);													//GPIO alternative for PD0
	GPIOD->MODER |= (1<<3);														//GPIO alternative for PD1
	GPIOD->MODER &= ~(1<<2);													//GPIO alternative for PD1
	GPIOD->MODER |= (1<<9);														//GPIO alternative for PD4
	GPIOD->MODER &= ~(1<<8);													//GPIO alternative for PD4
	GPIOD->MODER |= (1<<11);													//GPIO alternative for PD5
	GPIOD->MODER &= ~(1<<10);													//GPIO alternative for PD5
	GPIOD->MODER |= (1<<15);													//GPIO alternative for PD7
	GPIOD->MODER &= ~(1<<14);													//GPIO alternative for PD7
	GPIOD->MODER |= (1<<17);													//GPIO alternative for PD8
	GPIOD->MODER &= ~(1<<16);													//GPIO alternative for PD8
	GPIOD->MODER |= (1<<19);													//GPIO alternative for PD9
	GPIOD->MODER &= ~(1<<18);													//GPIO alternative for PD9
	GPIOD->MODER |= (1<<21);													//GPIO alternative for PD10
	GPIOD->MODER &= ~(1<<20);													//GPIO alternative for PD10
	GPIOD->MODER |= (1<<29);													//GPIO alternative for PD14
	GPIOD->MODER &= ~(1<<28);													//GPIO alternative for PD14
	GPIOD->MODER |= (1<<31);													//GPIO alternative for PD15
	GPIOD->MODER &= ~(1<<30);													//GPIO alternative for PD15
	GPIOE->MODER |= (1<<15);													//GPIO alternative for PE7
	GPIOE->MODER &= ~(1<<14);													//GPIO alternative for PE7
	GPIOE->MODER |= (1<<17);													//GPIO alternative for PE8
	GPIOE->MODER &= ~(1<<16);													//GPIO alternative for PE8
	GPIOE->MODER |= (1<<19);													//GPIO alternative for PE9
	GPIOE->MODER &= ~(1<<18);													//GPIO alternative for PE9
	GPIOE->MODER |= (1<<21);													//GPIO alternative for PE10
	GPIOE->MODER &= ~(1<<20);													//GPIO alternative for PE10
	GPIOE->MODER |= (1<<23);													//GPIO alternative for PE11
	GPIOE->MODER &= ~(1<<22);													//GPIO alternative for PE11
	GPIOE->MODER |= (1<<25);													//GPIO alternative for PE12
	GPIOE->MODER &= ~(1<<24);													//GPIO alternative for PE12
	GPIOE->MODER |= (1<<27);													//GPIO alternative for PE13
	GPIOE->MODER &= ~(1<<26);													//GPIO alternative for PE13
	GPIOE->MODER |= (1<<29);													//GPIO alternative for PE14
	GPIOE->MODER &= ~(1<<28);													//GPIO alternative for PE14
	GPIOE->MODER |= (1<<31);													//GPIO alternative for PE15
	GPIOE->MODER &= ~(1<<30);													//GPIO alternative for PE15
	GPIOF->MODER |= (1<<1);														//GPIO alternative for PF0
	GPIOF->MODER &= ~(1<<0);													//GPIO alternative for PF0

	//Set FSMC GPIOs to very high speed
	GPIOD->OSPEEDR |= (1<<1);
	GPIOD->OSPEEDR |= (1<<0);
	GPIOD->OSPEEDR |= (1<<3);
	GPIOD->OSPEEDR |= (1<<2);
	GPIOD->OSPEEDR |= (1<<9);
	GPIOD->OSPEEDR |= (1<<8);
	GPIOD->OSPEEDR |= (1<<11);
	GPIOD->OSPEEDR |= (1<<10);
	GPIOD->OSPEEDR |= (1<<15);
	GPIOD->OSPEEDR |= (1<<14);
	GPIOD->OSPEEDR |= (1<<17);
	GPIOD->OSPEEDR |= (1<<16);
	GPIOD->OSPEEDR |= (1<<19);
	GPIOD->OSPEEDR |= (1<<18);
	GPIOD->OSPEEDR |= (1<<21);
	GPIOD->OSPEEDR |= (1<<20);
	GPIOD->OSPEEDR |= (1<<29);
	GPIOD->OSPEEDR |= (1<<28);
	GPIOD->OSPEEDR |= (1<<31);
	GPIOD->OSPEEDR |= (1<<30);
	GPIOE->OSPEEDR |= (1<<15);
	GPIOE->OSPEEDR |= (1<<14);
	GPIOE->OSPEEDR |= (1<<17);
	GPIOE->OSPEEDR |= (1<<16);
	GPIOE->OSPEEDR |= (1<<19);
	GPIOE->OSPEEDR |= (1<<18);
	GPIOE->OSPEEDR |= (1<<21);
	GPIOE->OSPEEDR |= (1<<20);
	GPIOE->OSPEEDR |= (1<<23);
	GPIOE->OSPEEDR |= (1<<22);
	GPIOE->OSPEEDR |= (1<<25);
	GPIOE->OSPEEDR |= (1<<24);
	GPIOE->OSPEEDR |= (1<<27);
	GPIOE->OSPEEDR |= (1<<26);
	GPIOE->OSPEEDR |= (1<<29);
	GPIOE->OSPEEDR |= (1<<28);
	GPIOE->OSPEEDR |= (1<<31);
	GPIOE->OSPEEDR |= (1<<30);
	GPIOF->OSPEEDR |= (1<<1);
	GPIOF->OSPEEDR |= (1<<0);

	//Select AF12 on GPIOs
	GPIOD->AFR[0] |= (12<<0);													//AF12 for PD0
	GPIOD->AFR[0] |= (12<<4);													//AF12 for PD1
	GPIOD->AFR[0] |= (12<<16);													//AF12 for PD4 (NWE)
	GPIOD->AFR[0] |= (12<<20);													//AF12 for PD5 (NOE)
	GPIOD->AFR[0] |= (12<<28);													//AF12 for PD7 (NE1)
	GPIOD->AFR[1] |= (12<<0);													//AF12 for PD8
	GPIOD->AFR[1] |= (12<<4);													//AF12 for PD9
	GPIOD->AFR[1] |= (12<<8);													//AF12 for PD10
	GPIOD->AFR[1] |= (12<<24);													//AF12 for PD14
	GPIOD->AFR[1] |= (12<<28);													//AF12 for PD15
	GPIOE->AFR[0] |= (12<<28);													//AF12 for PE7
	GPIOE->AFR[1] |= (12<<0);													//AF12 for PE8
	GPIOE->AFR[1] |= (12<<4);													//AF12 for PE9
	GPIOE->AFR[1] |= (12<<8);													//AF12 for PE10
	GPIOE->AFR[1] |= (12<<12);													//AF12 for PE11
	GPIOE->AFR[1] |= (12<<16);													//AF12 for PE12
	GPIOE->AFR[1] |= (12<<20);													//AF12 for PE13
	GPIOE->AFR[1] |= (12<<24);													//AF12 for PE14
	GPIOE->AFR[1] |= (12<<28);													//AF12 for PE15
	GPIOF->AFR[0] |= (12<<0);													//AF12 for PF0 (A0)

	//Set control GPIOs
	//LCD_reset
	GPIOD->MODER |= (1<<22);													//GPIO output for PD11
	GPIOD->MODER &= ~(1<<23);													//GPIO output for PD11

	//Backlight
	//this is a PWM to control the brightness of the attached screen
	GPIOF->MODER |= (1<<10);													//GPIO output for PF5
	GPIOF->MODER &= ~(1<<11);													//GPIO output for PF5

	GPIOF->ODR |= (1<<5);														//we put a constant HIGH signal on the backlight


	//Set FSMC clocking
	RCC->AHB3RSTR |= (1<<0);													//reset FSMC
	RCC->AHB3RSTR &= ~(1<<0);
	RCC->AHB3ENR |= (1<<0);														//enable FSMC

	//Configure FMSC
	//Note: FSMC_Bank1 is the general register, not just for Bank1
	//Note: BCR and BTR are merged into an array of 8 registers (2 for each of the 4 banks)
	//BCR
	FSMC_Bank1->BTCR[0] = 0x0;
	FSMC_Bank1->BTCR[0] |= (1<<12);												//WREN
	FSMC_Bank1->BTCR[0] |= (1<<4);												//16-bit bus width
	FSMC_Bank1->BTCR[0] &= ~(1<<5);
	FSMC_Bank1->BTCR[0] |= (1<<7);												//reserved, must be kept at 1
	FSMC_Bank1->BTCR[0] |= (1<<0);												//bank enable

	//BTR
	//Note: timing is selected for 100 MHz AHB!
	FSMC_Bank1->BTCR[1] = 0x0;
	FSMC_Bank1->BTCR[1] |= (3<<0);												//address setup 3
	FSMC_Bank1->BTCR[1] |= (15<<4);												//max address hold
	FSMC_Bank1->BTCR[1] |= (3<<8);												//data phase 3
	FSMC_Bank1->BTCR[1] |= (2<<16);												//bus turn around 2
	FSMC_Bank1->BTCR[1] |= (15<<20);											//1/16 prescale
	FSMC_Bank1->BTCR[1] |= (15<<24);											//max latency

}


void LCD_IO_Init(void){
	/*
	 * This is just resetting the LCD
	 * We toggle the reset pin of the screen
	 *
	 */

	GPIOD->BSRR |= (1<<27);
	Delay_ms(5);
	GPIOD->BSRR |= (1<<11);
	Delay_ms(10);

}

void LCD_IO_WriteReg(uint8_t Reg){
	/*
	 * This is to write the register to the LCD driver.
	 * The FMSC will send over the register/command written to the designated memory address
	 * Mind, we write to the memory bank but the pushing of the data will be done automatically
	 */

	*FMSC_BANK1_REG = Reg;

}


void LCD_IO_WriteData(uint16_t RegValue){
	/*
	 * This is to write one 16-bit data packet to the LCD driver.
	 * The FMSC will send over the data packet written to the designated memory address
	 * Mind, we write to the memory bank but the pushing of the data will be done automatically
	 */

	*FMSC_BANK1_DATA = RegValue;

}



uint16_t LCD_IO_ReadData(void){
	/*
	 * We read out the FMSC memory section where the data is stored
	 *
	 */

	return *FMSC_BANK1_DATA;

}

void LCD_IO_WriteMultipleData(uint16_t *pData, uint32_t Size){
	/*
	 * Multiple execution of the data writing function
	 *
	 */

	for(uint32_t i=0; i<Size; i++){

		LCD_IO_WriteData(pData[i]);

	}

}


void LCD_IO_Delay(uint32_t delay){

	Delay_ms(delay);

}


void GenerateImage(void){

	/*
	 * Generate a complex image
	 */

	//advanced image - 240 pattern
	for(int i = 0; i < 57600; i++){

		//mimicked after a similar pattern generated for the FPGA project

		uint8_t W;

		if(((i/2) / 240) == ((i/2) % 240)) {

			W = 0xFF;

		} else {

			W = 0x0;

		}

		uint8_t A;

		if(((((i/2) / 240) & 0x20) == 0x20) && ((((i/2) % 240) & 0x20) == 0x20)) {

			A = 0xFF;

		} else {

			A = 0x0;

		}

		uint8_t pat_43;

		if(  ((((i/2) / 240) >> 3) & 0x3) == ((((i/2) % 240) >> 3) & 0x3)  ) {

			pat_43 = 0xFF;

		} else {

			pat_43 = 0xC0;

		}

		uint8_t red = ((((((i/2) % 240) & 0x1F) & (pat_43)) << 2) | W );						//5-bits of RED
		uint8_t green = ((((i/2) % 240) & A ) | W);												//6-bits of GREEN
		uint8_t blue = (((i/2) / 240) | W) & 0x1F;												//5-bits of BLUE - DONE

		image[i] = ((uint16_t)((green >> 5) | (red << 3))<<8) | (green << 5) | blue;

	}

}

void WipeImage(void){

	for(int i = 0; i < 57600; i++){

		image[i] = 0x0;

	}

}
