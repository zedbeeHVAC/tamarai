/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */

/* USER CODE BEGIN 0 */
#include "config.h"
#include "mqtt_priv.h"
#include "mqtt.h"
#include "lwip_mqtt.h"
#include "dns.h"
#include "TCPServer.h"
#include "TFTP_FlashConf.h"
#include "cjson_packet.h"

static struct stTFTP_Network_Params LwipNetworkParms;
//#include "mqtt_interface.h"
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */
static ip4_addr_t brokerServerAddress;
struct mqtt_client_s mqtt_client;
typedef enum {
  SMTP_IDLE,
  SMTP_NAME_RESOLVING,
  SMTP_NAME_RESOLVED,
} smtp_state_t;

typedef enum {
  SMTP_ERR_NONE,
  SMTP_ERR_UNKNOWN_HOST,
} smtp_error_t;

typedef struct {
  smtp_state_t state;
  const char *serverName;
  ip_addr_t serverIP;
  smtp_error_t lastError;
} smtp_t;

static void smtp_serverFound(const char *name, const ip_addr_t *ipaddr, void *arg) {
  smtp_t *smtp = (smtp_t*)arg;

  if ((ipaddr) && (ipaddr->addr)) {
    ip4_addr_copy(smtp->serverIP, *ipaddr);
    smtp->lastError = SMTP_ERR_NONE;
    smtp->state = SMTP_NAME_RESOLVED;
    return;
  } else {
    smtp->lastError = SMTP_ERR_UNKNOWN_HOST;
  }
  smtp->state = SMTP_IDLE;
}

static void InitSMTP(smtp_t *smtp) {
  smtp->state = SMTP_IDLE;
  smtp->serverName = "";
  IP4_ADDR(&smtp->serverIP, 0, 0, 0, 0);
  smtp->lastError = SMTP_ERR_NONE;
}

static int GetHostAddress(struct netif *netifp, const char *hostName, ip4_addr_t *addr) {
  /* see http://lwip.wikia.com/wiki/DNS */
  err_t res;
  ip_addr_t resolved;
  ip4_addr_t gateway;
  smtp_t smtp;

  InitSMTP(&smtp);
  IP4_ADDR(&gateway, COFNIG_DNS_ADDR0, COFNIG_DNS_ADDR1, COFNIG_DNS_ADDR2, COFNIG_DNS_ADDR3);
  dns_setserver(0, &gateway);
  smtp.serverName = hostName;
  smtp.state = SMTP_NAME_RESOLVING;
  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,(" DNS resolving for: %s\r\n", smtp.serverName));
  res = dns_gethostbyname(smtp.serverName, &resolved, smtp_serverFound, &smtp);
  if (res==ERR_INPROGRESS) {
    while(smtp.state == SMTP_NAME_RESOLVING) {
      ethernetif_input(netifp);
      sys_check_timeouts(); /* Handle all system timeouts for all core protocols */
    }
  }
  if (smtp.state == SMTP_NAME_RESOLVED) {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,(" Address resolved to: %u.%u.%u.%u\r\n", ((u8_t *)&smtp.serverIP)[0], ((u8_t *)&smtp.serverIP)[1],
            ((u8_t *)&smtp.serverIP)[2], ((u8_t *)&smtp.serverIP)[3]));
    ip4_addr_copy(*addr, smtp.serverIP); /* copy */
    return 0; /* ok */
  } else if (smtp.lastError == SMTP_ERR_UNKNOWN_HOST) {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,(" Unknown host: %s\r\n", smtp.serverName));
    return -1; /* failure */
  }
  return -1; /* failure */
}
/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

/* USER CODE BEGIN 2 */
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
uint8_t SERVER_IP[4];
uint8_t* GetIP(void)
{
   return IP_ADDRESS;
}
uint8_t* GetSubnetMask(void)
{
   return NETMASK_ADDRESS;
}
uint8_t* GetGetway(void)
{
  return GATEWAY_ADDRESS;
}
uint8_t* GetServerIP(void)
{
  return SERVER_IP;
}
/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  populateparams();
  fvTFTP_NetworkParamsRead(&LwipNetworkParms);
  IP_ADDRESS[0] = LwipNetworkParms.ipaddress[0];
  IP_ADDRESS[1] = LwipNetworkParms.ipaddress[1];
  IP_ADDRESS[2] = LwipNetworkParms.ipaddress[2];
  IP_ADDRESS[3] = LwipNetworkParms.ipaddress[3];
  NETMASK_ADDRESS[0] = LwipNetworkParms.subnetmask[0];
  NETMASK_ADDRESS[1] = LwipNetworkParms.subnetmask[1];
  NETMASK_ADDRESS[2] = LwipNetworkParms.subnetmask[2];
  NETMASK_ADDRESS[3] = LwipNetworkParms.subnetmask[3];
  GATEWAY_ADDRESS[0] = LwipNetworkParms.gateway[0];
  GATEWAY_ADDRESS[1] = LwipNetworkParms.gateway[1];
  GATEWAY_ADDRESS[2] = LwipNetworkParms.gateway[2];
  GATEWAY_ADDRESS[3] = LwipNetworkParms.gateway[3];
   SERVER_IP[0] = LwipNetworkParms.serverip[0];
  SERVER_IP[1] = LwipNetworkParms.serverip[1];
  SERVER_IP[2] = LwipNetworkParms.serverip[2];
  SERVER_IP[3] = LwipNetworkParms.serverip[3];
  /* Initilialize the LwIP stack without RTOS */
  lwip_init();

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) without RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

/* USER CODE BEGIN 3 */
  //StartNetworkInterface();
  uint32_t Server_IPS = 0;
      for (int8_t i=3; i>=0; i--)
    {
      Server_IPS |= ( (LwipNetworkParms.serverip[i] & 0xFF ) << (i*8) );
    }
   brokerServerAddress.addr = Server_IPS;//LwipNetworkParms.serverip;//421988618;//3355486400;//421988618;//2951279225;
      //if (GetHostAddress(&gnetif, CONFIG_BROKER_HOST_NAME, &brokerServerAddress)!=0) {
      //LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("ERROR: unable to get IP address for broker!\r\n"));
      //LED1_On(); /* red error LED */
     // for(;;){}
      
    //}
    mqtt_do_connect(&mqtt_client,&brokerServerAddress);
  tcpServer_init();
/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
 * ----------------------------------------------------------------------
 * Function given to help user to continue LwIP Initialization
 * Up to user to complete or change this function ...
 * Up to user to call this function in main.c in while (1) of main(void) 
 *-----------------------------------------------------------------------
 * Read a received packet from the Ethernet buffers 
 * Send it to the lwIP stack for handling
 * Handle timeouts if LWIP_TIMERS is set and without RTOS
 * Handle the llink status if LWIP_NETIF_LINK_CALLBACK is set and without RTOS 
 */
void MX_LWIP_Process(void)
{
/* USER CODE BEGIN 4_1 */
/* USER CODE END 4_1 */
  ethernetif_input(&gnetif);
  
/* USER CODE BEGIN 4_2 */
/* USER CODE END 4_2 */  
  /* Handle timeouts */
  sys_check_timeouts();

/* USER CODE BEGIN 4_3 */
/* USER CODE END 4_3 */
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */
	
  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */	
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */	
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
