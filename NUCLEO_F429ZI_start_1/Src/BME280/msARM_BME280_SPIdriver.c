/*
 * msARM_BME280_SPIdriver.c
 *
 * Created: 22.10.2017 16:29:21
 *  Author: maras
 */ 

#include "msARM_BME280_SPIdriver.h"


int8_t msBME280_SPIinit(struct bme280_dev * dev)
{
	int8_t rslt = BME280_OK;
	//Konfiguracja sprzêtowa uk³adu
	dev->id = MSBME280_SPI_CS;
	dev->interface = BME280_SPI_INTF;
	dev->read = msBME280_SPIread;
	dev->write = msBME280_SPIwrite;
	dev->delay_ms = msBME280_SPIdelay_ms;
	
	rslt = bme280_init(dev); //Inicjalizacja sprzetowa ukladu
	
	//Konfiguracja pracy sensorów
	dev->settings.osr_h = MSBME280_SETT_OSRH;
	dev->settings.osr_t = MSBME280_SETT_OSRT;
	dev->settings.osr_p = MSBME280_SETT_OSRP;
	dev->settings.filter = MSBME280_SETT_FILTER;
	dev->settings.standby_time = MSBME280_SETT_STANDBY;
	
	rslt = bme280_set_sensor_settings(MSBME280_SEL_SETT, dev);
	rslt = bme280_set_sensor_mode(MSBME280_MODE, dev);
	
	HAL_Delay(10);
	return rslt;
}

int8_t msBME280_SPIread(uint8_t dev_id, uint8_t reg_addrcommand, uint8_t *data, uint16_t len)
{
	HAL_GPIO_WritePin(MSBME280_SPI_CS_Port, MSBME280_SPI_CS_Pin, GPIO_PIN_RESET);    		//Wybierz uklad
	HAL_SPI_Transmit(&MSBME280_SPI, &reg_addrcommand, 1, 500);								//Wypchnij adres/komendê
	HAL_SPI_Receive(&MSBME280_SPI, data, len, 500);											//Odczytaj wskazany pakiet
	HAL_GPIO_WritePin(MSBME280_SPI_CS_Port, MSBME280_SPI_CS_Pin, GPIO_PIN_SET); 			//Zwolnij uklad
	return MSBME280_SPI.ErrorCode;															//Zwroc status SPI
}

int8_t msBME280_SPIwrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	/*************************************************************************************/
	/* Uwaga: driver boscha przy zapisie typu burst, przeplata w pakiecie danych         */
	/* adres rejstru z danymi, ale bez adresu pierwszego rejestru reg_addr				 */
	/* wiec konieczne jest najpierw wyslanie adresu poczatkowego						 */
	/* rejestru (reg_addr), a potem przygotowanego przez driver pakietu danych	(*data)	 */
	/*************************************************************************************/
	
	HAL_GPIO_WritePin(MSBME280_SPI_CS_Port, MSBME280_SPI_CS_Pin, GPIO_PIN_RESET);    		//Wybierz uklad
	HAL_SPI_Transmit(&MSBME280_SPI, &reg_addr, 1, 500);										//Zapisz adres 1-go rejestru
	HAL_SPI_Transmit(&MSBME280_SPI, data, len, 500);										//Zapisz wskazany pakiet danych
	HAL_GPIO_WritePin(MSBME280_SPI_CS_Port, MSBME280_SPI_CS_Pin, GPIO_PIN_SET);  			//Zwolnij uklad
	return MSBME280_SPI.ErrorCode;															//Zwroc status SPI
}

void msBME280_SPIdelay_ms(uint32_t period)
{
	HAL_Delay(period);
}

double msBME280_SPIconv_pressure(struct bme280_data * data, double h_npm)
{
	volatile double temp, press, h_prim, press_tmp, press_avg, temp_avg;
	temp = (double) data->temperature / 100;	//Temperatura w stopniach
	press = (double) data->pressure / 10000;	//Cisnienie w hPa
	
	h_prim = 8000 * ((1 + 0.004 * temp) / press);			//Stopien baryczny
	press_tmp = press + (h_npm / h_prim);					//Pierwsze przyblizenie cisnienia na poziomie morza
	press_avg = (press + press_tmp) / 2;					//Srednie cisnienie warstwy pomiedzy punktem a morzem
	temp_avg = ((2 * temp) + ((0.6 * h_npm) / 100)) / 2;	//Srednia temperatura warstwy pomiedzy punktem a morzem
	h_prim = 8000 * ((1 + 0.004 * temp_avg) / press_avg);	//Dokladniejszy stopien baryczny
	return press + (h_npm / h_prim);						//return dokladne cisnienia na poziomie morza
}
