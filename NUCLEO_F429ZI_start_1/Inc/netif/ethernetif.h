/*
 * ethernetif.h
 *
 *  Created on: 26.11.2017
 *      Author: maras
 */

#ifndef NETIF_ETHERNETIF_H_
#define NETIF_ETHERNETIF_H_

#include "lwip/netif.h"

#define LED_LINK_UP HAL_GPIO_WritePin(LED2_BLUE_GPIO_Port, LED2_BLUE_Pin, true)
#define LED_LINK_DOWN HAL_GPIO_WritePin(LED2_BLUE_GPIO_Port, LED2_BLUE_Pin, false)

void ethernetif_set_link(struct netif *netif);

#endif /* NETIF_ETHERNETIF_H_ */
