/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/

#include "tftp_parse.h"
#include "Timer.h"
#include "TFTP_FlashConf.h"
typedef  void* (*open)(const char* fname, const char* mode, u8_t write);
typedef  void (*close)(void* handle);
typedef  int (*read)(void* handle, void* buf, int bytes);
typedef  int (*write)(void* handle, struct pbuf* p);
static const struct tftp_context tftp = {
  tftp_open,
  tftp_close,
  tftp_read,
  tftp_write
};
void
tftp_server_init()
{
  tftp_init(&tftp);
}

void*
tftp_open(const char* fname, const char* mode, u8_t write)
{
  if(strcmp(fname,"config.txt") != 0)
  {
    return 0;
  }
  if(strcmp(mode,"octet") != 0)
  {
    return 0;
  }
  if( write != 1 )
  {
    return 0;
  }
}

void
tftp_close(void* handle)
{
  return;
}

int
tftp_read(void* handle, void* buf, int bytes)
{
  return 0;
}

char *revbuf;

int
tftp_write(void* handle, struct pbuf* p)
{
  
  if( p->tot_len <= 0)
  {
    return 0;
  }
  revbuf = p->payload;
  char *token = revbuf;
  
  /* get the first token */
  token = strtok(revbuf, ",");
   while ((token = strtok(revbuf, "\r\n")) != NULL ) {
        //flash_data(tok);
        CmdDecoder(token);
	revbuf = NULL;
    }
    
    SetAddresses();
  return 0;
}



