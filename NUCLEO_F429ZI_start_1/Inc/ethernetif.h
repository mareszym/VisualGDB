/*
 * ethernetif.h
 *
 *  Created on: 21.11.2017
 *      Author: maras
 */

#ifndef ETHERNETIF_H_
#define ETHERNETIF_H_
void ethernetif_input(struct netif *netif);
err_t ethernetif_init(struct netif *netif);
void ethernetif_set_link(struct netif *netif);
void ethernetif_update_config(struct netif *netif);
void ethernet_transmit(void);


#endif /* ETHERNETIF_H_ */
