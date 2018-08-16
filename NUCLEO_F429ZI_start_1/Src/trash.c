/*
 * trash.c
 *
 *  Created on: 17.11.2017
 *      Author: maras
 */

//#ifdef BUILD_TRASH
				  
		spi_txarray[0] = 0xD0 | 0x80;  //Ustaw bit R/W w tryb R
		HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET); 	//Wybierz uk³ad
		HAL_Delay(1);
		HAL_SPI_TransmitReceive(&hspi2, spi_txarray, spi_rxarray, 4, 1000);
		HAL_Delay(1);
		HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);  		//Zwolnij uk³ad

		printf("BME280 Chip ID = 0x%02X / ", spi_rxarray[1]);		//Bajt odebrany po adresie jest w drugiej komórce
		msPrintf8Binary(spi_rxarray[1]);
		printf("\r\n");

		HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET);  	//Wybierz uk³ad
		HAL_Delay(1);
		HAL_SPI_Transmit(&hspi2, spi_txarray, 1,1000);
		HAL_SPI_Receive(&hspi2, spi_rxarray, 4, 1000);
		HAL_Delay(1);
		HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);   		//Zwolnij uk³ad

		printf("BME280 Chip ID = 0x%02X \r\n", spi_rxarray[0]); //Bajt odebrany po adresie jest w pierwszej komórce
		printf("BME280 Chip ID = 0x%02X \r\n", spi_rxarray[1]);
		printf("BME280 Chip ID = 0x%02X \r\n", spi_rxarray[2]);
		printf("BME280 Chip ID = 0x%02X \r\n", spi_rxarray[3]);


				uint8_t chTx = 0x00;
				uint8_t chRx = 0x00;

				HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET);	//Wybierz uk³ad
				//HAL_Delay(1);
				HAL_SPI_Transmit(&hspi1, &chTx, 1, 1000);
				HAL_SPI_Receive(&hspi1, &chRx, 1, 1000);
				//HAL_Delay(1);
				HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);		//Zwolnij uk³ad

				printf("ENC_Reg 0x%02X = 0x%02X / ", chTx, chRx);
				msPrintf8Binary(chRx);
				printf("\r\n");

				printf("ENC GetLinkStatus = %d\r\n",ENC_GetLinkStatus(&ETH_ENCHandle));

//#endif // BUILD_TRASH