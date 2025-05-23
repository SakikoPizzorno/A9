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

#include "I2C.h"

//todo FUNCTION HEADER
void I2C_init(void) {
	//CLOCK setup
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN); //Enable GPIOB Clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_I2C1EN); // Enable I2C1 clock
	//Pin configuration
	GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1); //Alt Func Mode
	GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9); //open drain
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8_1 | GPIO_OSPEEDR_OSPEED9_1);
	GPIOB->PUPDR &= (GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9); //no pull/pulldown

	// set AF4
	GPIOB->AFR[1] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4))); // Clear AFR[1] bits for pins 8 & 9
	GPIOB->AFR[1] |= ((4 << (0 * 4)) | (4 << (1 * 4)));      // Set AF4 for I2C1

	// Configure I2C
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // enable I2C bus clock
	I2C1->CR1   &= ~( I2C_CR1_PE );        // put I2C into reset (release SDA, SCL)
	I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
	I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
	I2C1->TIMINGR = 0x00303D5B;            // 16 MHz SYSCLK timing from CubeMX
	//I2C1->TIMINGR = 0x00000E14; // valid for 4 MHz MSI

	I2C1->CR2   |=  ( I2C_CR2_AUTOEND );   // auto send STOP after transmission
	I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
	I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C
}

/* -----------------------------------------------------------------------------
 * function : I2C_write( )
 * INs      : byte of data to write and 15-bit address to put
 * OUTs     : none
 * action   : TODO
 * -------------------------------------------------------------------------- */
void I2C_write(uint16_t mem_address, uint8_t tx_data) {
    // Set up the write transaction
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES);
    I2C1->CR2 |= (DEVICE_ADDRESS << 1); // SADD is 7-bit, left-shift by 1
    I2C1->CR2 |= (3 << I2C_CR2_NBYTES_Pos); // 2 address bytes + 1 data byte
    I2C1->CR2 &= ~I2C_CR2_RD_WRN; // Write mode
    I2C1->CR2 |= I2C_CR2_START;

    // Send MSB of address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (MEMORY_ADDRESS >> 8);

    // Send LSB of address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (MEMORY_ADDRESS & 0xFF);

    // Send data byte
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = DATA_SEND;

    // Wait for stop condition
    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR |= I2C_ICR_STOPCF;
}

/* -----------------------------------------------------------------------------
 * function : I2C_read( )
 * INs      : 15-bit address of where to read byte of data
 * OUTs     : byte of data read at given address
 * action   : TODO
 * -------------------------------------------------------------------------- */
uint8_t I2C_read(uint16_t address) {
    // WRITE phase to set address pointer
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RD_WRN);
    I2C1->CR2 |= (MEMORY_ADDRESS << 1);
    I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos); // 2 address bytes
    I2C1->CR2 &= ~I2C_CR2_RD_WRN; // Write mode
    I2C1->CR2 |= I2C_CR2_START;

    // Send MSB of address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (address >> 8);

    // Send LSB of address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (address & 0xFF);

    // Wait for transfer complete
    while (!(I2C1->ISR & I2C_ISR_TC));

    // READ phase
    I2C1->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES);
    I2C1->CR2 |= (MEMORY_ADDRESS << 1);
    I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos); // 1 byte to read
    I2C1->CR2 |= I2C_CR2_RD_WRN; // Read mode
    I2C1->CR2 |= I2C_CR2_START;

    // Wait for RX data
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    uint8_t data = I2C1->RXDR;

    // Wait for stop condition
    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR |= I2C_ICR_STOPCF;

    return data;
}
