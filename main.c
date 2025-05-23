/*/*******************************************************************************
* EE 329 A9 I2C EEPROM
*******************************************************************************
* @file : main.c
* @brief : I2C with EEPROM
* project : EE 329 S'24 Assignment 4
* authors : Andrew Daouda -- adaouda@calpoly.edu
* 			Seth Saxena -- stsaxena@calpoly.edu
* version : 0.2
* date : 240523
* compiler : STM32CubeCLT v.1.15.0
* target : NUCLEO-L496ZG
* @attention : (c) 2023 STMicroelectronics. All rights reserved.
******************************************************************************
* REVISION HISTORY
* 0.1 240429 Initial version
******************************************************************************
* main.c from Raheel Rehmatullah
* Accessed: 5/23/2024
*****************************************************************************/
#include "main.h"
#include "i2c.h"
#define DEVICE_ADDRESS 0x52
#define DATA_TO_SEND 0x77
void SystemClock_Config(void);
int main(void)
{
	// LED INIT for TIMER (New to A3)
		// configure GPIO pins PC4 for:
		// output mode, push-pull, no pull up or pull down, high speed
		GPIOC->MODER   &= ~(GPIO_MODER_MODE4); // MAKES BITS ZERO (MAKES INPUT MODE, DEFAULT_
		GPIOC->MODER   |=  (GPIO_MODER_MODE4_0); // specify output mode
		GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT4); // specify push-pull
		GPIOC->PUPDR   &= ~(GPIO_PUPDR_PUPD4); // make no pull up or pull down
		GPIOC->OSPEEDR |=  (3 << GPIO_OSPEEDR_OSPEED4_Pos); // high speed
		GPIOC->BRR = (GPIO_PIN_4); // preset PC4 to 0
//// enable blue on-board blue LED
//	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
//	GPIOB->MODER &= ~(GPIO_MODER_MODER7);
//	GPIOB->MODER |= GPIO_MODER_MODER7_0;
//	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR7);
//	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED7;
//	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);
//	GPIOB->BRR = (GPIO_PIN_7);
	HAL_Init();
	SystemClock_Config();
	I2C_init();
	//Write Data to EEPROM
	I2C_write(DEVICE_ADDRESS, DATA_TO_SEND, 0x05);
	//Delay
	for(int i = 0; i < 20000; i++);
	//Check if the data read is the same as received, if so, turn on the LED
	if(DATA_TO_SEND == (I2C_read( DEVICE_ADDRESS, 0x05))){
		GPIOB->BSRR = (GPIO_PIN_7);
	}
	while (1){
	}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
