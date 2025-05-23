/*
* i2c.h
*
*  Created on: May 23, 2024
*      Author: andre
*/
#ifndef SRC_I2C_H_
#define SRC_I2C_H_
#include "stm32l4xx_hal.h"
void I2C_init(void);
void I2C_write(uint8_t device_address, uint8_t tx_data, uint16_t tx_address);
uint8_t I2C_read(uint8_t device_address, uint16_t rx_address);
#endif /* SRC_I2C_H_ */

