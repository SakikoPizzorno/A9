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

#ifndef SRC_I2C_H_
#define SRC_I2C_H_
#include "stm32l4xx_hal.h"
#include "main.h"
//#include "keypad.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

// define constants
#define DEVICE_ADDRESS 0x52
#define DATA_SEND 0xEF
#define MEMORY_ADDRESS 0xABCD
void SystemClock_Config(void);

// include function prototypes
void I2C_init(void);
void I2C_write(uint16_t mem_address, uint8_t tx_data);
uint8_t I2C_read(uint16_t rx_address);
#endif /* SRC_I2C_H_ */

