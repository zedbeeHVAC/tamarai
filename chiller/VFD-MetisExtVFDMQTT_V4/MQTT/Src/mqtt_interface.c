#include "config.h"
#include "lwip.h"
#include "lwip/apps/mqtt.h"
#include <string.h>
#include "lwip/dns.h"
#include "mqtt_interface.h"


typedef enum {
  MQTT_STATE_INIT,
  MQTT_STATE_IDLE,
  MQTT_STATE_DO_CONNECT,
#if MQTT_USE_TLS
  MQTT_STATE_DO_TLS_HANDSHAKE,
#endif
  MQTT_STATE_WAIT_FOR_CONNECTION,
  MQTT_STATE_CONNECTED,
  MQTT_STATE_DO_PUBLISH,
  MQTT_STATE_DO_SUBSCRIBE,
  MQTT_STATE_DO_DISCONNECT
} MQTT_State_t;

static MQTT_State_t MQTT_state = MQTT_STATE_INIT;

#if !CONFIG_USE_FREERTOS
static void DoMQTT_BM(struct netif *netifp, ip4_addr_t *broker_ipaddr) {
  uint32_t timeStampMs;
//  uint32_t diffTimeMs;
  uint32_t blinkTimeStampMs;
  //#define CONNECT_DELAY_MS   1000 /* delay in seconds for connect */
  //#define PUBLISH_PERIOD_MS  10000 /* publish period in seconds */
  #define BLINK_PERIOD_MS    2000

  timeStampMs = blinkTimeStampMs = sys_now(); /* get time in milli seconds */
  for(;;) {
   // diffTimeMs = sys_now()-timeStampMs;
    if (sys_now()-blinkTimeStampMs > BLINK_PERIOD_MS) {
      //LED2_Neg();
      blinkTimeStampMs = sys_now();
    }
#if 0
    if (MQTT_state==MQTT_STATE_IDLE && diffTimeMs>CONNECT_DELAY_MS) {
      MQTT_state = MQTT_STATE_DO_CONNECT; /* connect after 1 second */
      timeStampMs = sys_now(); /* get time in milli seconds */
    }
    if (MQTT_state==MQTT_STATE_CONNECTED && diffTimeMs>=PUBLISH_PERIOD_MS) {
      MQTT_state = MQTT_STATE_DO_PUBLISH; /* publish */
      timeStampMs = sys_now(); /* get time in milli seconds */
    }
#endif
    MqttDoStateMachine(&mqtt_client, broker_ipaddr); /* process state machine */
    ProcessLWIP(netifp);
  #if CONFIG_USE_SHELL
    SHELL_Process();
  #endif
  }
}
#endif

static int mqtt_do_connect(mqtt_client_t *client, ip4_addr_t *broker_ipaddr) {
  struct mqtt_connect_client_info_t ci;
  err_t err;

  memset(client, 0, sizeof(mqtt_client_t)); /* initialize all fields */

  /* Setup an empty client info structure */
  memset(&ci, 0, sizeof(ci));
  /* Minimal amount of information required is client identifier, so set it here */
  ci.client_id = CONFIG_CLIENT_ID_NAME;
  ci.client_user = CONFIG_CLIENT_USER_NAME;
  ci.client_pass = CONFIG_CLIENT_USER_PASSWORD;
  ci.keep_alive = 60; /* timeout */

  /* Initiate client and connect to server, if this fails immediately an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting
     to establish a connection with the server.
     For now MQTT version 3.1.1 is always used */
#if MQTT_USE_TLS
  client->ssl_context = &ssl;
  err = mqtt_client_connect(client, broker_ipaddr, MQTT_PORT_TLS, mqtt_connection_cb, 0, &ci);
#else
  err = mqtt_client_connect(client, broker_ipaddr, MQTT_PORT, mqtt_connection_cb, 0, &ci);
#endif
  /* For now just print the result code if something goes wrong */
  if(err != ERR_OK) {
    LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("mqtt_connect return %d\n", err));
    return -1; /* error */
  }
  return 0; /* ok */
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, "print_payload") == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* All topics starting with 'A' might be handled at the same way */
    inpub_id = 1;
  } else if(strcmp(topic, "placa/led2") == 0) {
	  inpub_id = 2;
  } else if(strcmp(topic, "placa/barra") == 0) {
	 inpub_id = 3;
  }
  else {
    /* For all other topics */
    inpub_id = 9;
  }

}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  printf("Incoming publish payload with length %d, flags %u\n", len, (unsigned int)flags);

  // }
  if(flags & MQTT_DATA_FLAG_LAST) {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0) {
      /* Don't trust the publisher, check zero termination */
      if(data[len-1] == 0) {
        printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
      }
    } else if(inpub_id == 1) {
      /* Call an 'A' function... */
    } else if(inpub_id == 2) {
    	if(strcmp(data, "0") == 0){ //Then, turn off LED3
    	}else if (strcmp(data, "1") == 0){ //Then turn on LED3
    	}
    	/* Call an 'A' function... */
    } else if(inpub_id == 3) {
/* Not yet done. It's suppossed to turn on/off */
    	if(strcmp(data, "0") == 0){ //Then, turn off LED2
    	      
    	    } else if(strcmp(data, "1") == 0){ //Then turn on LED2
    	    	
    	    } else if(strcmp(data, "2") == 0){ //Then turn on LED2
    	    	
    	    }
    	    //else if (strcmp(data, "3") == 0){ //Then turn on LED2
    	//    	Board_LED_Set(LEDS_LED1, false);
    	//    	Board_LED_Set(LEDS_LED2, false);
    	//    	Board_LED_Set(LEDS_LED3, false);
    	//    	Board_LED_Set(LEDS_LED4, true);
    	//    	Board_LED_Set(LEDS_LED5, true);
    	//    	Board_LED_Set(LEDS_LED6, true);
    	//    	Board_LED_Set(LEDS_LED7, true);
    	//    	Board_LED_Set(LEDS_LED8, true);
    	//    } else if (strcmp(data, "4") == 0){ //Then turn on LED2
    	//    	Board_LED_Set(LEDS_LED1, false);
    	//    	Board_LED_Set(LEDS_LED2, false);
    	//    	Board_LED_Set(LEDS_LED3, false);
    	//    	Board_LED_Set(LEDS_LED4, false);
    	//    	Board_LED_Set(LEDS_LED5, true);
    	//    	Board_LED_Set(LEDS_LED6, true);
    	//    	Board_LED_Set(LEDS_LED7, true);
    	//    	Board_LED_Set(LEDS_LED8, true);
    	//    }else if (strcmp(data, "5") == 0){ //Then turn on LED2
    	//    	Board_LED_Set(LEDS_LED1, false);
    	//    	Board_LED_Set(LEDS_LED2, false);
    	//    	Board_LED_Set(LEDS_LED3, false);
    	//    	Board_LED_Set(LEDS_LED4, false);
    	//    	Board_LED_Set(LEDS_LED5, false);
    	//    	Board_LED_Set(LEDS_LED6, true);
    	//    	Board_LED_Set(LEDS_LED7, true);
    	//    	Board_LED_Set(LEDS_LED8, true);
    	//    }else if (strcmp(data, "6") == 0){ //Then turn on LED2
    	//    	Board_LED_Set(LEDS_LED1, false);
    	//    	Board_LED_Set(LEDS_LED2, false);
    	//    	Board_LED_Set(LEDS_LED3, false);
    	//    	Board_LED_Set(LEDS_LED4, false);
    	//    	Board_LED_Set(LEDS_LED5, false);
    	//    	Board_LED_Set(LEDS_LED6, false);
    	//    	Board_LED_Set(LEDS_LED7, true);
    	//    	Board_LED_Set(LEDS_LED8, true);
    	//    }else if (strcmp(data, "7") == 0){ //Then turn on LED2
    	//    	Board_LED_Set(LEDS_LED1, false);
    	//    	Board_LED_Set(LEDS_LED2, false);
    	//    	Board_LED_Set(LEDS_LED3, false);
    	//    	Board_LED_Set(LEDS_LED4, false);
    	//    	Board_LED_Set(LEDS_LED5, false);
    	//    	Board_LED_Set(LEDS_LED6, false);
    	//    	Board_LED_Set(LEDS_LED7, false);
    	//    	Board_LED_Set(LEDS_LED8, true);
    	//    }else if (strcmp(data, "8") == 0){ //Then turn on LED2
    	//    	Board_LED_Set(LEDS_LED1, false);
    	//    	Board_LED_Set(LEDS_LED2, false);
    	//    	Board_LED_Set(LEDS_LED3, false);
    	//    	Board_LED_Set(LEDS_LED4, false);
    	//    	Board_LED_Set(LEDS_LED5, false);
    	//    	Board_LED_Set(LEDS_LED6, false);
    	//    	Board_LED_Set(LEDS_LED7, false);
    	//    	Board_LED_Set(LEDS_LED8, false);
    	//    }
      } else {
      printf("mqtt_incoming_data_cb: Ignoring payload...\n");
    }
  } else {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }







}
static void mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n", result);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  const char * topico = arg;
  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "placa" with QoS level 0, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, topico, 0, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n", err);
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);

    /* Its more nice to be connected, so try to reconnect */
    example_do_connect(client);
  }

}


void example_do_connect(mqtt_client_t *client)
{
  struct mqtt_connect_client_info_t ci;
  err_t err;
  
  /* Setup an empty client info structure */
  memset(&ci, 0, sizeof(ci));
  
  /* Minimal amount of information required is client identifier, so set it here */ 
  ci.client_id = "lwip_test";
  
  /* Initiate client and connect to server, if this fails immediately an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting 
     to establish a connection with the server. 
     For now MQTT version 3.1.1 is always used */
  ip_addr_t mqttServerIP;
  IP4_ADDR(&mqttServerIP, 192, 168, 0, 102);
  err = mqtt_client_connect(client, &mqttServerIP, MQTT_PORT, mqtt_connection_cb, 0, &ci);
  
  /* For now just print the result code if something goes wrong*/
  if(err != ERR_OK) {
    printf("mqtt_connect return %d\n", err);
  }
}

/* Called when publish is complete either with sucess or failure */
static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}
void example_publish(mqtt_client_t *client, void *arg)
{
  //const char *pub_payload= "Hola mundo de mierda!";
  const char *pub_payload= arg;
  err_t err;
  u8_t qos = 0; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  err = mqtt_publish(client, "placa", pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
}

void StartNetworkInterface(void) {
  uint8_t buf [32];
  struct netif fsl_netif0 = fnetifreturn();
  ip4_addr_t fsl_netif0_ipaddr = fipaddrreturn();
  ip4_addr_t fsl_netif0_netmask = fnetmaskrreturn();
  ip4_addr_t fsl_netif0_gw = fgwreturn();

#if CONFIG_USE_DHCP
  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("Starting DHCP....\r\n"));
  DHCP_Start(&fsl_netif0);
  while (DHCP_IsBound(&fsl_netif0)) {
    ProcessLWIP(&fsl_netif0);
    LED1_Neg();
  }
  LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("DHCP done!\r\n"));
#endif
if (CONFIG_BROKER_HOST_IP !=NULL) 
  {
    ip4_addr_set_u32(&brokerServerAddress, ipaddr_addr(CONFIG_BROKER_HOST_IP)); /* default init */
  }
#if CONFIG_USE_DNS
  if (CONFIG_BROKER_HOST_NAME!=NULL) {
    if (GetHostAddress(&fsl_netif0, CONFIG_BROKER_HOST_NAME, &brokerServerAddress)!=0) {
      LWIP_DEBUGF(MQTT_APP_DEBUG_TRACE,("ERROR: unable to get IP address for broker!\r\n"));
      //LED1_On(); /* red error LED */
      for(;;){}
    }
  }
#endif
#if !CONFIG_USE_DHCP
  ip4addr_ntoa_r(&fsl_netif0_ipaddr, (char*)buf, sizeof(buf));
  //CLS1_printf(" IPv4 Address     : %su\r\n", buf);
  ip4addr_ntoa_r(&fsl_netif0_netmask, (char*)buf, sizeof(buf));
  //CLS1_printf(" IPv4 Subnet mask : %s\r\n", buf);
  ip4addr_ntoa_r(&fsl_netif0_gw, (char*)buf, sizeof(buf));
  //CLS1_printf(" IPv4 Gateway     : %s\r\n", buf);
#endif
  ip4addr_ntoa_r(&brokerServerAddress, (char*)buf, sizeof(buf));
  //CLS1_printf(" Broker Address   : %s\r\n", buf);
  //CLS1_printf("************************************************\r\n");

#if CONFIG_USE_SNTP
  SNTP_Init();
#endif
}
#if CONFIG_USE_DNS
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
#endif
