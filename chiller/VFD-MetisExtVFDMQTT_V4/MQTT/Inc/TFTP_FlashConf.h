#ifndef __TFTP_FlashConf_H
#define __TFTP_FlashConf_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define TFTPPARMS_FLASH_START_ADDRESS            ((uint32_t)0x0803F800)//for TFTP params parameters page 127
#define TFTPPARMS_FLASH_END_ADDRESS            ((uint32_t)0x0803FFFF)
#define BOARDCONFIGSIZE   (70)
#define VALIDPASSCODE                       ((uint32_t)0x7547c7)
#define FLASH_ADDR_PASSCODE                 ((uint32_t)TFTPPARMS_FLASH_START_ADDRESS)
#define FLASH_ADDR_IPADDRESS                ((uint32_t)FLASH_ADDR_PASSCODE + 4)  
#define FLASH_ADDR_SUBNETMASK               ((uint32_t)FLASH_ADDR_IPADDRESS + 16)
#define FLASH_ADDR_GATEWAY                  ((uint32_t)FLASH_ADDR_SUBNETMASK + 16)
#define FLASH_ADDR_SERVERIP                 ((uint32_t)FLASH_ADDR_GATEWAY + 16)
#define FLASH_ADDR_UUID                     ((uint32_t)FLASH_ADDR_SERVERIP + 16)
   

struct stTFTP_Network_Params
{
    uint32_t passcode; //4
    uint32_t ipaddress[4]; //16 =20   
    uint32_t subnetmask[4]; //16 =36
    uint32_t gateway[4]; //16 = 52
    uint32_t serverip[4];  //4=56
    //uint32_t UUID;          //4=60
};
struct stTFTP_FRAM_Params
{
  uint32_t BTU;
  uint32_t Running_Hours;
  uint32_t UVRunning_Hours;
};
struct stTFTP_ONTime_OFFTime
{
  uint8_t ontime[2];
   uint8_t offtime[2];
uint8_t uvontime[2];
   uint8_t uvofftime[2];
   uint8_t  time[3];
};
struct stTFTP_Date
{
  uint16_t date[3];
  
};
void CmdDecoder(char *CmdString);
void ExtractCmdStr(char *string, char *sub_string);
void ExtractAddress(char *string, uint32_t *address);
void SetAddresses(void);
void FormAddress(char *temp, uint32_t *address);
void populateparams(void);
void getipaddress(uint32_t *ipaddressbagempty);
void getsubnetmask(uint32_t *subnetbagempty);
void getgateway(uint32_t *gatewaybagempty);
void getpasscode(uint32_t *passcodebagempty);
void getserverip(uint32_t *serveripbagempty);
void getUUID(uint32_t *UUIDbagempty);
uint8_t ExtractId(char *string, uint32_t *sub_string);
int FormId(char *temp, uint32_t *id);
void fvTFTP_NetworkParamsRead(struct stTFTP_Network_Params *TFTPParams);
void fvTFTP_NetworkParamsWrite(struct stTFTP_Network_Params *TFTPParams);
void ExtractOnTimeOffTime(char *string, uint8_t *address);
void FormOnTimeOffTime(char *temp, uint8_t *address);
void ExtractDate(char *string, uint16_t *address);
void FormDate(char *temp, uint16_t *address);
void ExtractTime(char *string, uint8_t *address);
void FormTime(char *temp, uint8_t *address);
#ifdef __cplusplus
}
#endif

#endif /* __TFTP_FlashConf_H */

