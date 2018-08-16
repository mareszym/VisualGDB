/*
 * msSTM32Function.c
 *
 *  Created on: 07.11.2017
 *      Author: maras
 */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "msSTM32Function.h"

/*********************************************************/
/* Definicje funkcji I/O potrzebnych do dzialania printf */
/*********************************************************/

int __io_putchar(int ch) {
	// Code to write character 'ch' on the UART
	HAL_UART_Transmit(PRINTF_CONSOLE, (uint8_t *)&ch, 1, 1000);
	return ch;
}

int __io_getchar(void) {
	// Code to read a character from the UART
	uint8_t ch;
	HAL_UART_Receive(PRINTF_CONSOLE, (uint8_t *)&ch, 1, 1000);
	return ch;
}

void msPrintf8Binary(uint8_t num){
	uint64_t mask = 0x0000000000000001;
	mask = mask << ((sizeof(num) * 8) - 1);
	for (uint8_t i = 0; i < (sizeof(num) * 8); i++)
	{
		if (num & mask)
			printf("1");
		else
			printf("0");			
		num <<= 1;
	}
}
void msPrintf16Binary(uint16_t num) {
	uint64_t mask = 0x0000000000000001;
	mask = mask << ((sizeof(num) * 8) - 1);
	for (uint8_t i = 0; i < (sizeof(num) * 8); i++)
	{
		if (((i % 8)==0) && (i!=0))
				{
					printf(" ");
				}

		if (num & mask)
			printf("1");
		else
			printf("0");			
		num <<= 1;
	}
}
void msPrintf32Binary(uint32_t num) {
	uint64_t mask = 0x0000000000000001;
	mask = mask << ((sizeof(num) * 8) - 1);
	for (uint8_t i = 0; i < (sizeof(num) * 8); i++)
	{
		if (((i % 8)==0) && (i!=0))
				{
					printf(" ");
				}

		if (num & mask)
			printf("1");
		else
			printf("0");			
		num <<= 1;
	}
}
void msPrintf64Binary(uint64_t num) {
	uint64_t mask = 0x0000000000000001;
	mask = mask << ((sizeof(num) * 8) - 1);
	for (uint8_t i = 0; i < (sizeof(num) * 8); i++)
	{
		if (((i % 8)==0) && (i!=0))
				{
					printf(" ");
				}

		if (num & mask)
			printf("1");
		else
			printf("0");			
		num <<= 1;
	}
}
void msStdTypesSize(void)
{
	printf("sizeof(char) = %d\n", sizeof(char));
	printf("sizeof(short) = %d\n", sizeof(short));
	printf("sizeof(int) = %d\n", sizeof(int));
	printf("sizeof(long) = %d\n", sizeof(long));
	printf("sizeof(long long) = %d\n", sizeof(long long));
	printf("sizeof(float) = %d\n", sizeof(float));
	printf("sizeof(double) = %d\n", sizeof(double));
	printf("sizeof(long double) = %d\n", sizeof(long double));
}




