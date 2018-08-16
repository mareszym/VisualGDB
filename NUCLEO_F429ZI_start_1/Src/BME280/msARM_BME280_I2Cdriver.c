/*
 * msARM_BME280_driver.c
 *
 * Created: 30.06.2017 20:21:06
 *  Author: maras
 *  
 */ 


#include "msARM_BME280_I2Cdriver.h"


int8_t msBME280_I2Cinit(struct bme280_dev * dev)
{
	int8_t rslt = BME280_OK;
		//Konfiguracja sprzêtowa uk³adu
		dev->id = MSBME280_ID;
		dev->interface = BME280_I2C_INTF;
		dev->read = msBME280_I2Cread;
		dev->write = msBME280_I2Cwrite;
		dev->delay_ms = msBME280_delay_ms;
		
		rslt = bme280_init(dev); //Inicjalizacja sprzetowa ukladu
		
		//Konfiguracja pracy sensorow
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

int8_t msBME280_I2Cread(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t dev_id_read;

	dev_id_read = ((dev_id<<1) | 0x01); //Oblicz adres ukladu w trybie read;
	return HAL_I2C_Mem_Read(&MSBME280_I2C, dev_id_read, reg_addr, 1, data, len, 100);
	//return MSBME280_I2C.ErrorCode;
}

int8_t msBME280_I2Cwrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	uint8_t dev_id_write;

	dev_id_write = (dev_id<<1); //Oblicz adres ukladu w trybie write;
	return HAL_I2C_Mem_Write(&MSBME280_I2C, dev_id_write, reg_addr, 1, data, len, 100);
}

void msBME280_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}

double msBME280_conv_pressure(struct bme280_data * data, double h_npm)
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
