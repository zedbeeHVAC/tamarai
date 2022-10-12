#ifndef __DATASTRUCTURE_H
#define __DATASTRUCTURE_H

#include "main.h"

#define MAXVAV                32

struct BoardConfig
{
  uint32_t passcode; //4
  uint32_t ipaddress[4]; //16 =20   
  uint32_t subnetmask[4]; //16 =36
  uint32_t gateway[4]; //16 = 52
  uint32_t macaddress[6]; //24 = 76
  uint32_t serverip[4];  //16 =92
  uint32_t date[3]; //12 = 104
  uint32_t time[3]; //12 =116
  uint32_t VAVID[MAXVAV]; //
  uint32_t maxvav; //120
  uint32_t VFDID; //124
  uint32_t flowmtrspn;
  uint32_t motrdir;
  uint32_t actuatrdir;
  uint32_t loopcntrl;
  uint32_t VFDUniqeIDChr[3];//152
  uint32_t VFDUniqeIDInt[4];//168
  uint32_t InstalationDate[3]; //180
  uint32_t PlaceCoordinates[4];//196
  uint32_t HostMode; //200
  uint32_t port_num; //204
  uint32_t u32aTime[4]; //220
  uint32_t u32aTimeOff[4] ; //236
  uint32_t TP_Control;//240
  uint32_t PID_Constant[2];//248
  uint32_t Min_Freq[2];//256 0th index for before decimal point first index for after decimal point
  uint32_t Max_Freq[2];//264 0th index for before decimal point first index for after decimal point  
}; //308

#define BOARDCONFIGSIZE   (264+4*MAXVAV)//(140+4*MAXVAV)

void
fvBoardParamsRead(struct BoardConfig *BoardConfig_data);

void
fvBoardParamsWrite(struct BoardConfig *BoardConfig_data);

#endif

