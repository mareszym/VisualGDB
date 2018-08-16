/*
 * msARM_BME280_driver.h
 *
 * Created: 30.06.2017 20:20:38
 *  Author: maras
 */ 


#ifndef MSARM_BME280_DRIVER_H_
#define MSARM_BME280_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "bme280_defs.h"
#include "bme280.h"

//Konfiguracja uk³adu
#define MSBME280_I2C hi2c2								//Port I2C, do ktorego podlaczony jest sensor

#define MSBME280_ID BME280_I2C_ADDR_PRIM					//Adres I2C ukladu BME280
//#define MSBME280_INTF BME280_I2C_INTF						//Typ interfejsu BME280 (I2C / SPI)

#define MSBME280_SETT_OSRH BME280_OVERSAMPLING_1X			//Oversampling pomiaru wilgotnosci
#define MSBME280_SETT_OSRT BME280_OVERSAMPLING_1X			//Oversampling pomiaru temperatury
#define MSBME280_SETT_OSRP BME280_OVERSAMPLING_1X			//Oversampling pomiaru cisnienia
#define MSBME280_SETT_FILTER BME280_FILTER_COEFF_16			//Wspolczynnik filtra dolnoprzepustowego
#define MSBME280_SETT_STANDBY BME280_STANDBY_TIME_500_MS	//Odstep pomiedzy pomiarami w trybie NORMAL

#define MSBME280_SEL_SETT (BME280_OSR_HUM_SEL | BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL) //Wybor rodzajow pomiarow, ktorych ustawienia
																							//maja byc wprowadzone
#define MSBME280_MODE BME280_NORMAL_MODE					//Tryb pracy ukladu

struct bme280_dev msBME280_I2Cdev;		//Struktura przechowujaca parametry ukladu
struct bme280_data msBME280_I2Cdata;	//Struktura przechowujaca dane pomiarowe
int8_t msBME280_rslt;				//Status wyjscia z funkcji

int8_t msBME280_I2Cinit(struct bme280_dev * dev); //Funkcja inicjujaca prace uk³adu

double msBME280_conv_pressure(struct bme280_data * data, double h_npm); //Konwersja cisnienia do wartosci znormalizowanej na poziomie morza
																		//h_npm - wysokosc nad poziomem morza

/****************************************************************************/
/* Deklaracje funkcji wg prototypow wymaganych przez driver bme280_defs.h   */
/****************************************************************************/

int8_t msBME280_I2Cread(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len); //Funkcja komunikacyjna - odczyt z BME280

int8_t msBME280_I2Cwrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len); //Funkcja komunikacyjna - zapis do ukladu

void msBME280_delay_ms(uint32_t period); //Funkcja opoznienia (nie mam pojecia po co, w zaszdzie nieuzywana)

#endif /* MSARM_BME280_DRIVER_H_ */
