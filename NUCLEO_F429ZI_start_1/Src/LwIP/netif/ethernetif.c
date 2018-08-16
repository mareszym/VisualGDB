/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"

/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Server/Src/ethernetif.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file implements Ethernet network interface drivers for lwIP
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "lwip/opt.h"
//#include "lwip/lwip_timers.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "../../ENC28J60/msENC28J60_HAL.h"
#include "../../ENC28J60/ENC28J60.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Network interface name */
#define IFNAME0 'e'
#define IFNAME1 'n'

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//ENC_HandleTypeDef EncHandle;
//SPI_HandleTypeDef hspi4;
#ifdef USE_PROTOTHREADS
static struct pt transmit_pt;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//
//#ifdef ENC28J60_INTERRUPT
//  /* EXTI interrupt init*/
//  /* Sets the priority grouping field */
//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
//  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
//  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
//#endif /* ENC28J60_INTERRUPT */
//
//}

/*******************************************************************************
					   LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
static void low_level_init(struct netif *netif)
{
  //uint8_t macaddress[6]= { MAC_ADDR0, MAC_ADDR1, MAC_ADDR2, MAC_ADDR3, MAC_ADDR4, MAC_ADDR5 };

  /* Initialize transmit protothread */
#ifdef USE_PROTOTHREADS
  PT_INIT(&transmit_pt);
#endif

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  ENC_MACADDR0;
  netif->hwaddr[1] =  ENC_MACADDR1;
  netif->hwaddr[2] =  ENC_MACADDR2;
  netif->hwaddr[3] =  ENC_MACADDR3;
  netif->hwaddr[4] =  ENC_MACADDR4;
  netif->hwaddr[5] =  ENC_MACADDR5;

  ENC_Handle.Init.MACAddr = netif->hwaddr;
  ENC_Handle.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
  ENC_Handle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  ENC_Handle.Init.InterruptEnableBits =  EIE_LINKIE;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

  /* Start the EN28J60 module */

  /*
   * @todo: Reset sprzêtowy ENC
   * zrobiæ warunkowo zale¿nie od #define HDWR_RST
   */

  if (ENC_Start(&ENC_Handle)) {
	/* Set the MAC address */
	ENC_SetMacAddr(&ENC_Handle);
  }
}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	/* TODO use netif to check if we are the right ethernet interface */
  err_t errval;
  struct pbuf *q;
  uint32_t framelength = 0;

#ifdef USE_PROTOTHREADS
if (EncHandle.transmitLength != 0) {
	 while (PT_SCHEDULE(ENC_Transmit(&transmit_pt, &ETH_ENCHandle))) {
		 /* Wait for end of previous transmission */
	 }
  }
#endif

  /* Prepare ENC28J60 Tx buffer */
  errval = ENC_RestoreTXBuffer(&ENC_Handle, p->tot_len);
  if (errval != ENC_ERR_OK) {
	  return errval;
  }

  /* copy frame from pbufs to driver buffers and send packet */
  for(q = p; q != NULL; q = q->next) {
	ENC_WriteBuffer(q->payload, q->len);
	framelength += q->len;
  }

  if (framelength != p->tot_len) {
	 return ENC_ERR_BUF;
  }

  ENC_Handle.transmitLength = p->tot_len;
  /* Actual transmission is triggered in main loop */

  return ENC_ERR_OK;
}

/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;
  struct pbuf *q;
  uint16_t len;
  uint8_t *buffer;
  uint32_t bufferoffset = 0;

  if (!ENC_GetReceivedFrame(&ENC_Handle)) {
	return NULL;
  }

  /* Obtain the size of the packet and put it into the "len" variable. */
  len = ENC_Handle.RxFrameInfos.length;
  buffer = (uint8_t *)ENC_Handle.RxFrameInfos.buffer;

  if (len > 0)
  {
	/* We allocate a pbuf chain of pbufs from the Lwip buffer pool */
	p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
  }

  if (p != NULL)
  {
	bufferoffset = 0;

	for(q = p; q != NULL; q = q->next)
	{
	  /* Copy data in pbuf */
	  memcpy( (uint8_t*)((uint8_t*)q->payload), (uint8_t*)((uint8_t*)buffer + bufferoffset), q->len);
	  bufferoffset = bufferoffset + q->len;
	}
  }

  return p;
}

/**
  * @brief This function should be called when a packet is ready to be read
  * from the interface. It uses the function low_level_input() that
  * should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{
  err_t err;
  struct pbuf *p;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);

  /* no packet could be read, silently ignore this */
  if (p == NULL) return;

  /* entry point to the LwIP stack */
  err = netif->input(p, netif);

  if (err != ENC_ERR_OK)
  {
	LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
	pbuf_free(p);
	p = NULL;
  }
}

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "stm32idisco";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ENC_ERR_OK;
}

/**
  * @brief  Returns the current time in milliseconds
  *         when LWIP_TIMERS == 1 and NO_SYS == 1
  * @param  None
  * @retval Current Time value
  */
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/**
  * @brief  This function sets the netif link status.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_set_link(struct netif *netif)
{
	/* Handle ENC28J60 interrupt */
	ENC_IRQHandler(&ENC_Handle);

	/* Check whether the link is up or down*/
	if(((ENC_Handle.LinkStatus) & PHSTAT2_LSTAT)!= 0) {
	  netif_set_link_up(netif);
	  HAL_GPIO_WritePin(LED2_BLUE_GPIO_Port, LED2_BLUE_Pin, true);
 //     printf("ENC from DOWN to UP\r\n");
	} else {
	  netif_set_link_down(netif);
//      printf("ENC from UP to DOWN\r\n");
	  HAL_GPIO_WritePin(LED2_BLUE_GPIO_Port, LED2_BLUE_Pin, false);
	}

	/* Reenable interrupts */
	ENC_EnableInterrupts(EIE_INTIE);
}

void ethernetif_notify_conn_changed(struct netif *netif)
{
	if (netif_is_link_up(netif))
	{
		printf("netif status changed to UP: %s\r\n", ip4addr_ntoa(netif_ip4_addr(netif)));
	} else
	{
		printf("netif status changed to DOWN\r\n");
	}

}

/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
void ethernetif_update_config(struct netif *netif)
{
  if(netif_is_link_up(netif)) {
	  /* Restart the EN28J60 module */
	  low_level_init(netif);
  }
ethernetif_notify_conn_changed(netif);
}

/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */

//__weak void ethernetif_notify_conn_changed(struct netif *netif)
//{
//  /* NOTE : This is function could be implemented in user file
//            when the callback is needed,
//  */
//}

/**
 * Implement actual transmission triggering
 */
void ethernet_transmit(void) {
#ifdef USE_PROTOTHREADS
	ENC_Transmit(&transmit_pt, &ETH_ENCHandle);
#else
	ENC_Transmit(&ENC_Handle);
#endif
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#if 0 /* don't build, this is only a skeleton, see previous comment */

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "lwip/ethip6.h"
#include "lwip/etharp.h"
#include "netif/ppp/pppoe.h"

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
};

/* Forward declarations. */
static void  ethernetif_input(struct netif *netif);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
  struct ethernetif *ethernetif = netif->state;

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] = ;
  ...
  netif->hwaddr[5] = ;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

#if LWIP_IPV6 && LWIP_IPV6_MLD
  /*
   * For hardware/netifs that implement MAC filtering.
   * All-nodes link-local is handled by default, so we must let the hardware know
   * to allow multicast packets in.
   * Should set mld_mac_filter previously. */
  if (netif->mld_mac_filter != NULL) {
	ip6_addr_t ip6_allnodes_ll;
	ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
	netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_ADD_MAC_FILTER);
  }
#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

  /* Do whatever else is needed to initialize interface. */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
  struct ethernetif *ethernetif = netif->state;
  struct pbuf *q;

  initiate transfer();

#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  for (q = p; q != NULL; q = q->next) {
	/* Send the data from the pbuf to the interface, one pbuf at a
	   time. The size of the data in each pbuf is kept in the ->len
	   variable. */
	send data from(q->payload, q->len);
  }

  signal that packet should be sent();

  MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
  if (((u8_t*)p->payload)[0] & 1) {
	/* broadcast or multicast packet*/
	MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
  } else {
	/* unicast packet */
	MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
  }
  /* increase ifoutdiscards or ifouterrors on error */

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  LINK_STATS_INC(link.xmit);

  return ENC_ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *
low_level_input(struct netif *netif)
{
  struct ethernetif *ethernetif = netif->state;
  struct pbuf *p, *q;
  u16_t len;

  /* Obtain the size of the packet and put it into the "len"
	 variable. */
  len = ;

#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

  if (p != NULL) {

#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

	/* We iterate over the pbuf chain until we have read the entire
	 * packet into the pbuf. */
	for (q = p; q != NULL; q = q->next) {
	  /* Read enough bytes to fill this pbuf in the chain. The
	   * available data in the pbuf is given by the q->len
	   * variable.
	   * This does not necessarily have to be a memcpy, you can also preallocate
	   * pbufs for a DMA-enabled MAC and after receiving truncate it to the
	   * actually received size. In this case, ensure the tot_len member of the
	   * pbuf is the sum of the chained pbuf len members.
	   */
	  read data into(q->payload, q->len);
	}
	acknowledge that packet has been read();

	MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
	if (((u8_t*)p->payload)[0] & 1) {
	  /* broadcast or multicast packet*/
	  MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
	} else {
	  /* unicast packet*/
	  MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
	}
#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

	LINK_STATS_INC(link.recv);
  } else {
	drop packet();
	LINK_STATS_INC(link.memerr);
	LINK_STATS_INC(link.drop);
	MIB2_STATS_NETIF_INC(netif, ifindiscards);
  }

  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
static void
ethernetif_input(struct netif *netif)
{
  struct ethernetif *ethernetif;
  struct eth_hdr *ethhdr;
  struct pbuf *p;

  ethernetif = netif->state;

  /* move received packet into a new pbuf */
  p = low_level_input(netif);
  /* if no packet could be read, silently ignore this */
  if (p != NULL) {
	/* pass all packets to ethernet_input, which decides what packets it supports */
	if (netif->input(p, netif) != ENC_ERR_OK) {
	  LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
	  pbuf_free(p);
	  p = NULL;
	}
  }
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
ethernetif_init(struct netif *netif)
{
  struct ethernetif *ethernetif;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

  ethernetif = mem_malloc(sizeof(struct ethernetif));
  if (ethernetif == NULL) {
	LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
	return ERR_MEM;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->state = ethernetif;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
  netif->linkoutput = low_level_output;

  ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);

  /* initialize the hardware */
  low_level_init(netif);

  return ENC_ERR_OK;
}

#endif /* 0 */
