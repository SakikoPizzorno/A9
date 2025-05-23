/*/****************************************************************************
* EE 329 A9 I2C EEPROM
*******************************************************************************
* @file : i2c.c
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
* i2c.c from Raheel Rehmatullah
* Accessed: 5/23/2024
*****************************************************************************/
#include "i2c.h"
/*
* PB8 -> i2c SCL
* PB9 -> i2c SDA
*/
/*
* I2C_init initialized the i2c GPIO and peripherals.
* Run it before your while loop
*/
void I2C_init(void){
	/* USER configure GPIO pins for I2C alternate functions SCL and SDA */
	// Configure I2C
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);   // enable GPIOB clock
	GPIOB->MODER &= ~(GPIO_MODER_MODE7 |
				GPIO_MODER_MODE8 |
				GPIO_MODER_MODE9); //clear bits
	GPIOB->MODER |= (GPIO_MODER_MODE8_1 |
				GPIO_MODER_MODE9_1); //set to output mode
	GPIOB->AFR[1] &= ~((0x000F << GPIO_AFRH_AFSEL8_Pos)
			| (0x000F << GPIO_AFRH_AFSEL9_Pos));
	GPIOB->AFR[1] |= ((0x0004 << GPIO_AFRH_AFSEL8_Pos)
			| (0x0004 << GPIO_AFRH_AFSEL9_Pos));	//Set to AF I2C mode
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7 |
				GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9 ); //Clear bits
	GPIOB->OTYPER |= (GPIO_OTYPER_OT8 |
				GPIO_OTYPER_OT9); //Set to Open Drain output
	GPIOB->PUPDR &= (GPIO_PUPDR_PUPD8 |
				GPIO_PUPDR_PUPD9); //Clear bits
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // enable I2C bus clock
	I2C1->CR1   &= ~( I2C_CR1_PE );  /* put I2C into reset
(release SDA, SCL) */
	I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
	I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
	I2C1->TIMINGR = 0x00303D5B;			   // set 14 MHz
	I2C1->CR2   |=  ( I2C_CR2_AUTOEND );   // auto send STOP after tx
	I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
	I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C
}
/*
* Write to your device
* Params include:
* Device address - the i2c address of the device
* tx_data - the data youre writing to the device
* tx_address - the register address that you want to send the data to
*/

void I2C_write(uint8_t device_address, uint8_t tx_data, uint16_t tx_address) {
    // Set write mode, clear byte count, enable AUTOEND
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;  // Set to write mode
    I2C1->CR2 &= ~(0xFF << I2C_CR2_NBYTES_Pos);  // Clear NBYTES
    I2C1->CR2 |= (3 << I2C_CR2_NBYTES_Pos);      // 3 bytes: 2 address, 1 data
    I2C1->CR2 |= I2C_CR2_AUTOEND;                // Automatically send STOP

    // Set 7-bit address (left-shifted)
    I2C1->CR2 &= ~I2C_CR2_SADD;
    I2C1->CR2 |= (device_address << 1); // SADD[7:1] = address, SADD[0] = don't care

    // Start I2C write operation
    I2C1->CR2 |= I2C_CR2_START;

    // Transmit MSB of memory address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = (tx_address >> 8) & 0xFF;

    // Transmit LSB of memory address
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = tx_address & 0xFF;

    // Transmit data
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = tx_data;

    // Wait for STOP condition and clear it
    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR |= I2C_ICR_STOPCF;
}


//void I2C_write(uint8_t device_address, uint8_t tx_data, uint16_t tx_address){
//	// build EEPROM transaction
//	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
//	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
//	I2C1->CR2   |=  ( 3 << I2C_CR2_NBYTES_Pos); //8 write 3 bytes (2 Addr,1 data)
//	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
//	I2C1->CR2   |=  ( device_address << (I2C_CR2_SADD_Pos) ); /* device addr SHL 1 */
//	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op
//	//^ Sends Device address
//	//Send address
//	while (!(I2C1->ISR & I2C_ISR_TXIS));
//	I2C1->TXDR = (tx_address >> 8) & 0xFF; // Address high byte
//	while (!(I2C1->ISR & I2C_ISR_TXIS));
//	I2C1->TXDR = tx_address & 0x00FF; // Address low byte
//	// Send the data to write
//	while (!(I2C1->ISR & I2C_ISR_TXIS));
//	I2C1->TXDR = tx_data;
//	// Wait for the STOP condition
//	while (!(I2C1->ISR & I2C_ISR_STOPF));
//	// Reset and re-enable I2C bus
//	//I2C1->CR1 &= ~( I2C_CR1_PE);
//	//I2C1->CR1 |= ( I2C_CR1_PE);
//}
/*
* Read from your device
* Params include:
* device_address - the i2c address of the device
* rx_address - the register address that you want to receive the data from
*/
uint8_t I2C_read(uint8_t device_address, uint16_t rx_address){
	// build EEPROM Write transaction
	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 2 << I2C_CR2_NBYTES_Pos); // write 2 bytes (2 addr)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	I2C1->CR2   |=  ( device_address << (I2C_CR2_SADD_Pos) ); /* device addr
SHL 1 */
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op
	//^ Sends Device address
	//Send Address
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = (rx_address >> 8) & 0xFF; // Address high byte
	while (!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = rx_address & 0x00FF; // Address low byte
	while (!(I2C1->ISR & I2C_ISR_STOPF));
	//delay
	//for(int i = 0; i < 10000; i++);
	// build EEPROM Read transaction
	I2C1->CR2   |=  ( I2C_CR2_RD_WRN );    // set READ mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 1 << I2C_CR2_NBYTES_Pos); // Read 1 byte (1 data)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	I2C1->CR2   |=  ( device_address << (I2C_CR2_SADD_Pos) );/* device addr
 											SHL 1 */
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op
	//^ Sends Device address
	//Recieve Data
	while (!(I2C1->ISR & I2C_ISR_RXNE));
	return (I2C1->RXDR); //Return data captured
}
