/*
 * msENC28J60_HAL.c
 *
 * Created: 30.10.2017 12:55:25
 *  Author: maras
 */ 

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include "enc28j60.h"
#include "msENC28J60_HAL.h"

/* Callback  functions  *********************************************************/

/**
  * SPI Slave selection and deselection.
  * param  select: true if the ENC28J60 slave SPI if selected, false otherwise
  * retval none
  */

void ENC_SPI_Select(bool select)
{
	if (select)
	{
		HAL_NVIC_DisableIRQ(ENC_IRQ);							//Wylacz przerwanie
		HAL_GPIO_WritePin(ENC_SPI_CS_PORT, ENC_SPI_CS_PIN, GPIO_PIN_RESET);		//Wybierz uklad
		up_udelay(1);
	} else
	{
		HAL_GPIO_WritePin(ENC_SPI_CS_PORT, ENC_SPI_CS_PIN, GPIO_PIN_SET);		//Zwolnij uklad
		up_udelay(1);
		HAL_NVIC_EnableIRQ(ENC_IRQ);							//Wlacz przerwanie
	}
}

/**
  * SPI single byte send and receive.
  * The ENC28J60 slave SPI must already be selected and wont be deselected after transmission
  * Must be provided by user code
  * param  command: command or data to be sent to ENC28J60
  * retval answer from ENC28J60
  */

uint8_t ENC_SPI_SendWithoutSelection(uint8_t command)
{
	return HAL_SPI_Transmit(ENC_SPI, &command, 1, 500);
}

/**
  * SPI single byte send and receive. Must be provided by user code
  * param  command: command or data to be sent to ENC28J60
  * retval answer from ENC28J60
  */

uint8_t ENC_SPI_Send(uint8_t command)
{
	HAL_StatusTypeDef status;
	
	ENC_SPI_Select(true);									//Wybierz uklad
	status = HAL_SPI_Transmit(ENC_SPI, &command, 1, 500);	//Wyslij dane
	ENC_SPI_Select(false);									//Zwolnij uklad
	return status;											//Zwroc status
}

/**
  * Implement SPI buffer send and receive. Must be provided by user code
  * param  master2slave: data to be sent from host to ENC28J60, can be NULL if we only want to receive data from slave
  * param  slave2master: answer from ENC28J60 to host, can be NULL if we only want to send data to slave
  * retval none
  */

void ENC_SPI_SendBuf(uint8_t *master2slave, uint8_t *slave2master, uint16_t bufferSize)
{
	ENC_SPI_Select(true);																//Wybierz uklad

	/* Transmit or receuve data */
    if (slave2master == NULL) {
        if (master2slave != NULL) {
            HAL_SPI_Transmit(ENC_SPI, master2slave, bufferSize, 1000);
        }
    } else if (master2slave == NULL) {
        HAL_SPI_Receive(ENC_SPI, slave2master, bufferSize, 1000);
    } else {
        HAL_SPI_TransmitReceive(ENC_SPI, master2slave, slave2master, bufferSize, 1000);
    }
	ENC_SPI_Select(false);																//Zwolnij uklad
}


