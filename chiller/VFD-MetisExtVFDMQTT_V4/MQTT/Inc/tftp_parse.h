/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include "tftp_server.h"


void
tftp_server_init();

void*
tftp_open(const char* fname, const char* mode, u8_t write);

void
tftp_close(void* handle);

int
tftp_read(void* handle, void* buf, int bytes);

int
tftp_write(void* handle, struct pbuf* p);

