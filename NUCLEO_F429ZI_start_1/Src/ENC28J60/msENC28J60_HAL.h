/*
 * msENC28J60_HAL.h
 *
 * Created: 30.10.2017 12:55:09
 *  Author: maras
 */ 


#ifndef MSENC28J60_HAL_H_
#define MSENC28J60_HAL_H_

#include "enc28j60.h"

#define ENC_SPI &hspi1
#define ENC_SPI_CS_PORT SPI1_CS0_GPIO_Port
#define ENC_SPI_CS_PIN SPI1_CS0_Pin
#define ENC_SPI_DEV_ID 0
#define ENC_IRQ EXTI9_5_IRQn

#define ENC_MACADDR0 0xDE
#define ENC_MACADDR1 0x7F
#define ENC_MACADDR2 0xFF
#define ENC_MACADDR3 0x07
#define ENC_MACADDR4 0xE3
#define ENC_MACADDR5 0xC0

ENC_HandleTypeDef ENC_Handle;
uint8_t ENC_MACAddress[6];


#endif /* MSENC28J60_HAL_H_ */
