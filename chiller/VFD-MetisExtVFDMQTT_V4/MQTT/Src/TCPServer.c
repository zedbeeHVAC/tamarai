#include "TCPServer.h"
#include "ModbusProcess.h"
#include "lwip/tcp.h"

err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
void close_my_conn( struct tcp_pcb *pcb );

void tcpServer_init(void)
{
  struct tcp_pcb *pcb;	            		
  
  /* Create a new TCP control block  */
  pcb = tcp_new();	                		 	

  /* Assign to the new pcb a local IP address and a port number */
  /* Using IP_ADDR_ANY allow the pcb to be used by any local interface */
  tcp_bind(pcb, IP_ADDR_ANY, 502);       


  /* Set the connection to the LISTEN state */
    
  pcb = tcp_listen(pcb);				

  /* Specify the function to be called when a connection is established */	
  tcp_accept(pcb, tcp_server_accept);   
										
}

/**
  * @brief  This funtion is called when a TCP connection has been established on the port TCP_PORT.
  * @param  arg	user supplied argument 
  * @param  pcb	the tcp_pcb which accepted the connection
  * @param  err error value
  * @retval ERR_OK
  */
err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{ 
  /* Specify the function that should be called when the TCP connection receives data */
  tcp_recv(pcb, tcp_server_recv);  
  
  return ERR_OK;  
}

/**
  * @brief  This function is called when a data is received over the TCP_PORT.
  *         The received data contains the number of the led to be toggled.
  * @param  arg	user supplied argument 
  * @param  pcb	the tcp_pcb which accepted the connection
  * @param  p the packet buffer that was received
  * @param  err error value
  * @retval ERR_OK
  */
err_t error;
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  uint16_t length;
  uint16_t totallength;    
  uint16_t size=0,i;  
  uint8_t data[200];   
  uint8_t *pkt_to_send; 
  
  if(err == ERR_OK && p != NULL)
  {    
    struct pbuf *q=p;
    
    length = q->len;
    totallength = q->tot_len; 
    tcp_recved(tpcb, totallength);
    
    u8_t *totaldata;
    totaldata = q->payload;
    for(i=0; i<length; i++) 
    {
      data[i] = totaldata[i];
    }    

    modbus_packet parsed_packet = mb_parsepacket(data, length);
   
    for(i=0;i<200;i++)
      stat_to_send[i]=0;
    
    
    if(parsed_packet.unitId == 128) // sending response for vfd ..
      pkt_to_send = mb_processpacketVFD(parsed_packet, &size);    
    else    // sending response for vav..
    {     
      pkt_to_send = mb_processpacketVAV(parsed_packet, &size);
    }     

    if (size!=0)        //If size==0, then tcp write won't happen.
      error = tcp_write(tpcb, pkt_to_send, size, 0);

    tcp_output(tpcb);
    tcp_sent(tpcb, NULL);
    pbuf_free(p);    
  }
  else
  { 
    close_my_conn(tpcb);
  }
  
  return err;

}

void close_my_conn( struct tcp_pcb *pcb )
{
  tcp_arg(pcb, NULL);
  tcp_sent(pcb, NULL);
  tcp_recv(pcb, NULL);
  tcp_close(pcb);
}