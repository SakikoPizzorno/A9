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

#ifndef __MAIN_H
#define __MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32l4xx_hal.h"
#include "stm32l4a6xx.h"

void Error_Handler(void);
#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
