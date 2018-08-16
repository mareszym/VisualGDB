/*
 * msARM_BME280_SPIdriver.h
 *
 * Created: 22.10.2017 16:29:06
 *  Author: maras
 *  
 *  Pod³¹czenie uk³adu BME po SPI:
 *  BME280	---		MCU
 *
 *  CSB		---		NPCSx
 *  SCK		---		SPCK
 *  SDI		---		MOSI
 *  SDO		---		MISO
 */ 


#ifndef MSARM_BME280_SPIDRIVER_H_
#define MSARM_BME280_SPIDRIVER_H_

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include "bme280_defs.h"
#include "bme280.h"

//Konfiguracja uk³adu
#define MSBME280_SPI hspi1									//Port SPI, do ktorego podlaczony jest sensor
#define MSBME280_SPI_CS 0									//SPI chip ID dom. NPCS0
#define MSBME280_SPI_CS_Port SPI1_CS0_GPIO_Port
#define MSBME280_SPI_CS_Pin SPI1_CS0_Pin

#ifndef MSBME280_SETT_OSRH
#define MSBME280_SETT_OSRH BME280_OVERSAMPLING_1X			//Oversampling pomiaru wilgotnosci
#endif
#ifndef MSBME280_SETT_OSRT
#define MSBME280_SETT_OSRT BME280_OVERSAMPLING_1X			//Oversampling pomiaru temperatury
#endif
#ifndef MSBME280_SETT_OSRP
#define MSBME280_SETT_OSRP BME280_OVERSAMPLING_1X			//Oversampling pomiaru cisnienia
#endif
#ifndef MSBME280_SETT_FILTER
#define MSBME280_SETT_FILTER BME280_FILTER_COEFF_16			//Wspolczynnik filtra dolnoprzepustowego
#endif
#ifndef MSBME280_SETT_STANDBY
#define MSBME280_SETT_STANDBY BME280_STANDBY_TIME_500_MS	//Odstep pomiêdzy pomiarami w trybie NORMAL
#endif
#ifndef MSBME280_SEL_SETT
#define MSBME280_SEL_SETT (BME280_OSR_HUM_SEL | BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL) //Wybor rodzajow pomiarow, ktorych ustawienia
#endif																						//maja byc wprowadzone
#ifndef MSBME280_MODE
#define MSBME280_MODE BME280_NORMAL_MODE					//Tryb pracy ukladu
#endif


struct bme280_dev msBME280_SPIdev;		//Struktura przechowujaca parametry ukladu
struct bme280_data msBME280_SPIdata;	//Struktura przechowujaca dane pomiarowe

typedef struct msBME280_spi_packet {				//Struktura pakietu danych do zapisu/odczytu SPI
	//! SPI address/commands to issue to the other chip (node).
	uint8_t addr;
	//! Length of the SPI data address segment (1-3 bytes).
	uint32_t addr_length;
	//! Where to find the data to be transferred.
	void *buffer;
	//! How many bytes do we want to transfer.
	uint32_t length;
	//! SPI chip address to communicate with.
	uint8_t chip;
} msBME280_spi_packet_t;



int8_t msBME280_SPIrslt;				//Status wyjscia z funkcji

int8_t msBME280_SPIinit(struct bme280_dev * dev); //Funkcja inicjujaca prace ukladu

double msBME280_SPIconv_pressure(struct bme280_data * data, double h_npm); //Konwersja cisnienia do wartosci znormalizowanej na poziomie morza
//h_npm - wysokosc nad poziomem morza

/****************************************************************************/
/* Deklaracje funkcji wg prototypow wymaganych przez driver bme280_defs.h   */
/****************************************************************************/

int8_t msBME280_SPIread(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len); //Funkcja komunikacyjna - odczyt z BME280

int8_t msBME280_SPIwrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len); //Funkcja komunikacyjna - zapis do uk³adu

void msBME280_SPIdelay_ms(uint32_t period); //Funkcja opoznienia (nie mam pojecia po co, w zaszdzie nieuzywana)




#endif /* MSARM_BME280_SPIDRIVER_H_ */
