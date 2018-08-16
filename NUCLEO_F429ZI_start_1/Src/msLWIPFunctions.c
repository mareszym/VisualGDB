#include "msLWIPFunctions.h"


bool msLWIP_ENC_link_state_changed(ENC_HandleTypeDef * enchandle, struct netif * netif)
{
	static bool LinkStatus;
	static bool prevLinkStatus;
	
	ENC_IRQHandler(enchandle);
	ENC_EnableInterrupts(EIE_INTIE);
	LinkStatus = ((enchandle->LinkStatus) & PHSTAT2_LSTAT) != 0;
	
	if (LinkStatus != prevLinkStatus)
	{
		if (LinkStatus)
		{
			netif_set_link_up(netif);
			printf("LINK UP\r\n");
			HAL_GPIO_WritePin(LED2_BLUE_GPIO_Port, LED2_BLUE_Pin, true);
		}
		else
		{
			netif_set_link_down(netif);
			printf("LINK DOWN\r\n");
			HAL_GPIO_WritePin(LED2_BLUE_GPIO_Port, LED2_BLUE_Pin, false);
		}
		prevLinkStatus = LinkStatus;
		return true;
	}
	return false;
} /* link_state_changed */

/**
  * @brief  Configurates the network interface
  * @param  None
  * @retval None
  */
void msLWIP_netif_config(struct netif * netif, netif_init_fn init, netif_input_fn input, netif_status_callback_fn link_callback)
{
	ip_addr_t ipaddr;
	ip_addr_t netmask;
	ip_addr_t gw;

	IP4_ADDR(&ipaddr, 192, 168, 0, 33);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
	IP4_ADDR(&gw, 192, 168, 0, 1); 

	/* add the network interface */
	netif_add(netif, &ipaddr, &netmask, &gw, NULL, init, input);

	/*  Registers the default network interface */
	netif_set_default(netif);

	netif_set_link_up(netif);

	if (netif_is_link_up(netif))
	{
		/* When the netif is fully configured this function must be called */
		netif_set_up(netif);
		//printf("ENC is UP\r\n");
	}
	else
	{
		/* When the netif link is down this function must be called */
		netif_set_down(netif);
		//printf("ENC is DOWN\r\n");
	}
	//dhcp_start(&ENC_netif);

	/* Set the link callback function, this function is called on change of link status*/
	netif_set_link_callback(netif, link_callback);
}

err_t my_tcp_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	printf("Connected !!!\r\n");
	//tcp_conn_OK = 1;
	return ERR_OK;
}
