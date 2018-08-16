#pragma once

#include "stm32f4xx_hal.h"

#include "lwip/dhcp.h"
#include "lwip/opt.h"
#include "lwip/ip_addr.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "netif/etharp.h"
#include "lwip/timeouts.h"
#include "ethernetif.h"

#include "ENC28J60\msENC28J60_HAL.h"
#include "ENC28J60\enc28j60.h"

/*
 * @brief	Funkcja testuje zmiane fizycznego linku
 *			i w razie jego zmiany wywoluje odpowiednia funkcje LWIP
 * @param	enchandle wskaznik do struktury interfejsu ENC28J60
 * @param	netif wskaznik do struktury interfejsu LWIP
 * @return	wynik sprawdzenia zmiana/brak zmiany
 * @retval	true/false
 * 
 */

bool msLWIP_ENC_link_state_changed(ENC_HandleTypeDef * ENChandle, struct netif * netif);

/*
  * @brief  Configurates the network interface
  * @param netif wskaznik do struktury interfejsu sieciowego
  * @param init callback function that initializes the interface
  * @param input callback function that is called to pass
  * ingress packets up in the protocol layer stack.\n
  * It is recommended to use a function that passes the input directly
  * to the stack (netif_input(), NO_SYS=1 mode) or via sending a
  * message to TCPIP thread (tcpip_input(), NO_SYS=0 mode).\n
  * These functions use netif flags NETIF_FLAG_ETHARP and NETIF_FLAG_ETHERNET
  * to decide whether to forward to ethernet_input() or ip_input().
  * In other words, the functions only work when the netif
  * driver is implemented correctly!\n
  * Most members of struct netif should be be initialized by the 
  * netif init function = netif driver (init parameter of this function).\n
  * IPv6: Don't forget to call netif_create_ip6_linklocal_address() after
  * setting the MAC address in struct netif.hwaddr
  * (IPv6 requires a link-local address).
  * @retval None
  */

void msLWIP_netif_config(struct netif * netif, netif_init_fn init, netif_input_fn input, netif_status_callback_fn link_callback);