/*
 * msSTM32Function.h
 *
 *  Created on: 07.11.2017
 *      Author: maras
 */

#ifndef MSSTM32FUNCTION_H_
#define MSSTM32FUNCTION_H_

#include <stdint.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#define PRINTF_CONSOLE &huart3 //Przekierowanie na wirtualny COM ST-LINK

void msPrintf8Binary(uint8_t num);
void msPrintf16Binary(uint16_t num);
void msPrintf32Binary(uint32_t num);
void msPrintf64Binary(uint64_t num);

void msStdTypesSize(void);

#endif /* MSSTM32FUNCTION_H_ */
