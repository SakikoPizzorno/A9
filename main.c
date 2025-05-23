/*******************************************************************************
* EE 329 A5 SPI Digital to Analog Converter (DAC)
*******************************************************************************
* @file           : main.c
* @brief          :
* project         : EE 329 S'25 Assignment 7
* authors         : Sakiko Pizzorno (spizzorn@calpoly.edu)
*					Alexander Von Fuchs
* version         : 0.1
* date            : 5/7/25
* compiler        : STM32CubeIDE v.1.12.0 Build: 14980_20230301_1550 (UTC)
* target          : NUCLEO-L4A6ZG (STM32L496ZG)
* clocks          : 4 MHz MSI to AHB2
* @attention      : (c) 2023 STMicroelectronics. All rights reserved.
*******************************************************************************/

#include "main.h"
#include "i2c.h"

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	I2C_init();

	//Write Data to EEPROM
	I2C_write(MEMORY_ADDRESS, DATA_SEND);

	//Delay
	for(int i = 0; i < 20000; i++);

	// Read Data Previously Written
	I2C_read(MEMORY_ADDRESS);

	//Check if the data read is the same as received, if so, turn on the LED
	if(DATA_SEND == (I2C_read(MEMORY_ADDRESS))){
		GPIOB->BSRR = (GPIO_PIN_7);
	}
	while (1){
	}
}

//Set up PB7, on-board LED
void LED_init(){
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);
	GPIOB-> MODER &= ~(GPIO_MODER_MODE7);
	GPIOB -> MODER |= (GPIO_MODER_MODE7_0); //Set PB0 to output mode
	GPIOB -> OTYPER &= ~(GPIO_OTYPER_OT7); // Set the OTYPER to push-pull
	GPIOB -> OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7);
	GPIOB -> OSPEEDR |= (GPIO_OSPEEDR_OSPEED7_1); //Set to high speed.
	GPIOB -> PUPDR &= ~(GPIO_PUPDR_PUPD7); //Set to no resistor
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
  //RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8; // sets MSI to 16 MHz
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
    SystemCoreClockUpdate();  // ✅ Place this LAST in the function

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
#endif
