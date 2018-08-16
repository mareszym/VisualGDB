
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "lwip/dhcp.h"
#include "lwip/opt.h"
#include "lwip/ip_addr.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "netif/etharp.h"
//#include "lwip/lwip_timers.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"
//#include "app_ethernet.h"
//#include "tcp_echoserver.h"

#include "msSTM32Function.h"
#include "msLWIPFunctions.h"

#include "BME280\msARM_BME280_SPIdriver.h"
#include "BME280\msARM_BME280_I2Cdriver.h"

#include "ENC28J60\msENC28J60_HAL.h"
#include "ENC28J60\enc28j60.h"

#include "msLWIP_testfn.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint64_t SysTick_Counter_A = 0;
//char message[] = "*** Hello Real World !!! *** \r\n";
int8_t bme280init_errorcode=0;
uint8_t i2cDATA=0;
volatile uint32_t last_tick=0;

//uint8_t spi_txarray[4] = { 0, 0, 0, 0 };
//uint8_t spi_rxarray[4] = { 0, 0, 0, 0 };

struct netif ENC_netif;
uint ETH_LinkStatus=0;

struct tcp_pcb * my_tcp_pcb;
struct ip4_addr my_dest_addr;

uint8_t my_message[]="HELLO TCP/IP WORLD !!!!";
uint8_t dhcp_addr_OK = 0;
uint8_t tcp_conn_OK = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("********** PROGRAM STARTING **************\r\n");

	/****** Inicjalizacja BME280 ******/

	//bme280init_errorcode = msBME280_SPIinit(&msBME280_SPIdev);
	bme280init_errorcode = msBME280_I2Cinit(&msBME280_I2Cdev);

	/* Hard reset ENC28J60 */
	HAL_GPIO_WritePin(ENC_RST_GPIO_Port, ENC_RST_Pin, false);
	HAL_Delay(10);
	HAL_GPIO_WritePin(ENC_RST_GPIO_Port, ENC_RST_Pin, true);
	HAL_Delay(1000);
	
	/* Network init*/

	lwip_init();
	msLWIP_netif_config(&ENC_netif,ethernetif_init, netif_input, ethernetif_update_config);

	ipaddr_aton("192.168.0.13",&my_dest_addr); //Ustaw adres zdalnego hosta do polaczenia

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		/* LWIP */
		
		/* Check link state, e.g. via MDIO communication with PHY */
		if (msLWIP_ENC_link_state_changed(&ENC_Handle, &ENC_netif))
		{
			if (netif_is_link_up(&ENC_netif))
			{
				netif_set_link_up(&ENC_netif);
			}
			else
			{
				netif_set_link_down(&ENC_netif);
			}
		}


		/* Read a received packet from the Ethernet buffers and send it
		to the lwIP for handling */

		ethernetif_input(&ENC_netif);
		
		/* Triggers actual ethernet transmission if transmission buffers are not empty */
		ethernet_transmit();

		/* Handle LWIP timeouts */
		sys_check_timeouts();
		
	 
		/* your application goes here */

		if ((HAL_GetTick() - last_tick) > 10000)
		{
			last_tick = HAL_GetTick();
			printf("\r\n ****** Hello NUCLEO 2 World !!! ****** \r\n\r\n");
			printf("System Tick = %d\r\n", (int)HAL_GetTick());


	//		msBME280_SPIrslt = bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &msBME280_SPIdata, &msBME280_SPIdev);
	//
	//		printf("SPI_Temperature    = %7.2f°C\r\n", (double)msBME280_SPIdata.temperature / 100);
	//		printf("SPI_Pressure / npm = %7.2fhPa / %0.2fhPa\r\n", (double)msBME280_SPIdata.pressure / 10000, msBME280_SPIconv_pressure(&msBME280_SPIdata, 120));
	//		printf("SPI_Humidity       = %7.2f%%\r\n", (double)msBME280_SPIdata.humidity / 1000);
	//		printf("\r\n");

			msBME280_SPIrslt = bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &msBME280_I2Cdata, &msBME280_I2Cdev);

			printf("I2C_Temperature    = %7.2f°C\r\n", (double)msBME280_I2Cdata.temperature / 100);
			printf("I2C_Pressure / npm = %7.2fhPa / %0.2fhPa\r\n", (double)msBME280_I2Cdata.pressure / 10000, msBME280_SPIconv_pressure(&msBME280_I2Cdata, 120));
			printf("I2C_Humidity       = %7.2f%%\r\n", (double)msBME280_I2Cdata.humidity / 1000);
			printf("\r\n");


			/************************************************************************/
			/* Test komunikacji ENC28J60                                            */
			/************************************************************************/

			printf("ENC EREVID = %d\r\n",ENC_GetEREVID(&ENC_Handle));
			printf("ENC GetLinkStatus = ");
			msPrintf16Binary(ENC_Handle.LinkStatus);
			printf("\r\n");
			printf("System Tick = %d\r\n", (int)HAL_GetTick());

			HAL_GPIO_TogglePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin);
			//HAL_GPIO_TogglePin(LED3_RED_GPIO_Port, LED3_RED_Pin);
		}

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

	/**Activate the Over-Drive mode 
	*/
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

	/**Configure the Systick interrupt time 
	*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick 
	*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
	  printf("HAL odszed³ do krainy wiecznych restartów :-( \r\n");
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
