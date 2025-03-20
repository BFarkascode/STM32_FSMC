/* USER CODE BEGIN Header */
/**
 *  Author: BalazsFarkas
 *  Project: STM32_TouchI2C_FMSC
 *  Processor: STM32F412ZG
 *  Compiler: ARM-GCC (STM32 IDE)
 *  Program version: 1.0
 *  File: main.c
 *  Hardware description/pin distribution:
 *  								I2C on PB6(SCL)/PB7(SDA)
 *  								CTP_RST on PF12
 *  								FSMC on PD0, PD1, PD4 (NWE),PD5 (NOE), PD7 (NE1), PD8, PD9, PD10,
 *  										PD14, PD15, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14,
 *  										PE15, PF0 (A0)
 *  								LCD_RST on PD11
 *  								LCD_Backlight on PF5
 *  Modified from: N/A
 *  Change history: N/A
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789h2.h"
#include "ft6x06.h"
#include "ClockDriver_STM32F4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t image[57600];																//image buffer in RAM
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
//  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  SysClockConfig();

  TIM6Config();																		//custom delay function

  FSMC_LCD_Init();

  ST7789H2_Init();

#ifdef publish_generated_image
  GenerateImage();
#endif

//#ifdef publish_a_drawing
  I2C1Config();
  ft6x06_Init(0x70);																//0x38<<1 = 0x70
  ft6x06_TS_Start(0x70);
  WipeImage();
//#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint16_t Xpos = 0;

	  uint16_t Ypos = 0;

	  uint16_t image_index = 0;

	  uint16_t RAW_X, RAW_Y = 0;

	  //Publish image
	  for(uint16_t i=0; i<240; i++){

		  //Note: here the publishing is in "horizontal" direction, not "vertical"

		  ST7789H2_SetCursor(0, Ypos);

		  ST7789H2_WriteReg(ST7789H2_WRITE_RAM, (uint8_t*)NULL, 0);

		  uint16_t counter = 0;

		  for(counter = 0; counter < 240; counter++)
		  {
		    LCD_IO_WriteData(image[image_index]);
		    image_index++;
		  }

		  Ypos++;

	  }

//#ifdef publish_a_drawing
	  //Detect a touch and update the drawing on the screen
	  if(ft6x06_TS_DetectTouch(0x70) == 1){											//if there is a touch detected

	  	ft6x06_TS_GetXY(0x70, &RAW_X, &RAW_Y);
	  	image[(57600 - RAW_X*240) + RAW_Y] = 0xFFFF;								//center of the touch
	  	image[(57600 - RAW_X*240) + RAW_Y + 1] = 0xFFFF;
	  	image[(57600 - RAW_X*240) + RAW_Y - 1] = 0xFFFF;
	  	image[(57600 - (RAW_X - 1)*240) + RAW_Y] = 0xFFFF;
	  	image[(57600 - (RAW_X - 1)*240) + RAW_Y + 1] = 0xFFFF;
	  	image[(57600 - (RAW_X - 1)*240) + RAW_Y - 1] = 0xFFFF;
	  	image[(57600 - (RAW_X + 1)*240) + RAW_Y] = 0xFFFF;
	  	image[(57600 - (RAW_X + 1)*240) + RAW_Y + 1] = 0xFFFF;
	  	image[(57600 - (RAW_X + 1)*240) + RAW_Y - 1] = 0xFFFF;

	  	image[(57600 - RAW_X*240) + RAW_Y + 2] = 0xFFFF;
	  	image[(57600 - RAW_X*240) + RAW_Y - 2] = 0xFFFF;
	  	image[(57600 - (RAW_X - 1)*240) + RAW_Y + 2] = 0xFFFF;
	  	image[(57600 - (RAW_X - 1)*240) + RAW_Y - 2] = 0xFFFF;
	  	image[(57600 - (RAW_X + 1)*240) + RAW_Y + 2] = 0xFFFF;
	  	image[(57600 - (RAW_X + 1)*240) + RAW_Y - 2] = 0xFFFF;

	  	image[(57600 - (RAW_X + 2)*240) + RAW_Y] = 0xFFFF;
	  	image[(57600 - (RAW_X + 2)*240) + RAW_Y + 1] = 0xFFFF;
	  	image[(57600 - (RAW_X + 2)*240) + RAW_Y - 1] = 0xFFFF;
	  	image[(57600 - (RAW_X + 2)*240) + RAW_Y + 2] = 0xFFFF;
	  	image[(57600 - (RAW_X + 2)*240) + RAW_Y - 2] = 0xFFFF;

	  	image[(57600 - (RAW_X - 2)*240) + RAW_Y] = 0xFFFF;
	  	image[(57600 - (RAW_X - 2)*240) + RAW_Y + 1] = 0xFFFF;
	  	image[(57600 - (RAW_X - 2)*240) + RAW_Y - 1] = 0xFFFF;
	  	image[(57600 - (RAW_X - 2)*240) + RAW_Y + 2] = 0xFFFF;
	  	image[(57600 - (RAW_X - 2)*240) + RAW_Y - 2] = 0xFFFF;

	  } else if(ft6x06_TS_DetectTouch(0x70) == 2){									//if there is two touches

		  WipeImage();																//we wipe the screen

	 } else {																		//if no touch happened

		 //do nothing

	  }
//#endif

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
