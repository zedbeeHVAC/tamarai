#ifndef __MODBUSPROCESS_H
#define __MODBUSPROCESS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "main.h"
/******************************************************************************/
#define MAXByteCoil                             1   //total no of bytes to be returned to bms. At present we have  4 coils so 1 it will take only 4 bits so 1 byte is enough
#define MB_DATA_LEN                            240//200
#define COILLENGTH              1
#define VAVINPUTLENGTH         220  // 24   
#define VAVHOLDLENGTH           6
#define VFDINPUTLEN            130//88//84// 56
#define VFDHOLDLEN             128//96//88 //68
#define ERROR_OFFSET            15
/************************Exception Code For MODBUS ************************/
#define ILLEGALFUNC                     0x01
#define ILLDATAADDRESS                  0x02
#define ILLDATAVALUE                    0x03
#define BMSERROROFFSET                  301
/******************************************************************************/
/***************************** REQUIRED REG ***********************************/
#define MAXCOILVAV                             1
#define MAXCOILVFD                             4
#define MAXINPUTVAVREG                         110//12
#define MAXINREGVFD                            58//44//42//28
#define MAXHREGVAV                             3
#define MAXHREGVFD                             64//48//44//34
/*******************function code**********************************************/
#define RDCOILSTAT                             0x01
#define RDHLDNGREG                             0X03
#define RDINREG                                0x04
#define WRONECOIL                              0x0F
#define WRHOLDING_REG                          0x10
#define READ_FILE_RECORD                       0x14

/******************************* VAV COIL (R/W)**********************************/
#define VAVCOILOFFSET                         0x0001
#define maVAVSTATUS                             0x0000
/****************************** VFD COIL (R/W) ********************************/
#define VFDCOILOFFSET                         0x0001
#define maVFDSTATUS                             0x0000
#define maSCHEDULE_ENABLE                       0x0001
#define maT_P_CONTROL                           0x0002
#define maH2O_Act_DIR                           0x0003
/****************************** VAV INPUT(R) **********************************/
#define VAVINPUTOFFSET                          0x7531
#define address1		0x0000
#define address2		0x0002
#define address3		0x0004
#define address4		0x0006
#define address5		0x0008
#define address6		0x000A
#define address7		0x000C
#define address8		0x000E
#define address9		0x0010
#define address10		0x0012
#define address11		0x0014
#define address12		0x0016
#define address13		0x0018
#define address14		0x001A
#define address15		0x001C
#define address16		0x001E
#define address17		0x0020
#define address18		0x0022
#define address19		0x0024
#define address20		0x0026
#define address21		0x0028
#define address22		0x002A
#define address23		0x002C
#define address24		0x002E
#define address25		0x0030
#define address26		0x0032
#define address27		0x0034
#define address28		0x0036
#define address29		0x0038
#define address30		0x003A
#define address31		0x003C
#define address32		0x003E
#define address33		0x0040
#define address34		0x0042
#define address35		0x0044
#define address36		0x0046
#define address37		0x0048
#define address38		0x004A
#define address39		0x004C
#define address40		0x004E
#define address41		0x0050
#define address42		0x0052
#define address43		0x0054
#define address44		0x0056
#define address45		0x0058
#define address46		0x005A
#define address47		0x005C
#define address48		0x005E
#define address49		0x0060
#define address50		0x0062
#define address51		0x0064
#define address52		0x0066
#define address53		0x0067
#define address54		0x0068
#define address55		0x0069
#define address56		0x006A
#define address57		0x006B
#define address58		0x006C
#define address59		0x006D

 
  
  


  
#define maAMBTEMP                               0x0000
#define maMODE                                  0x0001
#define maPRESSURE                              0x0003
#define maVAVDATE                               0x0005
#define maVAVTIME                               0x0007
#define maPIRSTATUS                             0x0009
#define maONOFFSOURCE                           0x000A
#define maSET_TEMP_SOURCE                       0x000B
/*******************************VFD INPUT(R) *********************************/

#define VFDINOFFSET                             0x7531
#define maRESERVED                              0x0000  //future use
#define maH20IN                                 0x0001
#define maH20OUT                                0x0003
#define maFLOWPERMIN                            0x0005
#define maRETURNAIR                             0x0007
#define maPOWER                                 0x0009
#define maBTUREADING                            0x000B
#define maC02LEVEL                              0x000D
#define maHUMIDITY                              0x000E
#define maFILTERLEVEL                           0x000F
#define maSUPPLYAIR                             0x0010
#define maDUCTPRESS                             0x0012
#define maVOLTAGE                               0x0014
#define maCURRENT                               0x0016
#define maRUNNING_HOURS                         0x0018
#define maFIRE_SIGNAL                           0x001A
#define maVFD_ONOFF_Source                      0x001B
#define maAQI                                   0x001C
#define maPM1p0                                 0x001D
#define maPM10                                  0x001E
#define maPM2p5                                 0x001F
#define matvoc                                  0x0020
#define maCO                                    0x0021
#define maNH3                                   0x0022
#define maNO2                                   0x0023
#define maAQC                                   0x0024
#define maLimitSwchStatus                       0x0025
#define maUVRunHours                            0x0026
#define maUVCurMeasured                         0x0028
#define maUVOffSource                           0x0029
#define maUVError                               0x002A
#define maFacOffSource                          0x002B
#define maWaterPressure                         0x002C

#define magpio1                                 0x002D
#define magpio2                                 0x002E
#define magpio3                                 0x002F
#define magpio4                                 0x0030
#define magpio5                                 0x0031
#define magpio6                                 0x0032
#define magpio7                                 0x0033
#define magpio8                                 0x0034
#define magpio9                                 0x0035
#define magpio10                                 0x0036
#define magpio11                                 0x0037
#define magpio12                                 0x0038
#define magpio13                                 0x0039
#define magpio14                                 0x003A

/******************************* VFD HOLDING (R/W)  *****************************/
#define VFDHOFFSET                              0x9C41
#define maFREQUENCY                             0x0000
#define maDIAGPACK                              0x0002
#define maMODESELEST                            0x0003
#define maSET_TEMP_VFD                          0x0004
#define maTFTP_RQST                             0x0006
#define maSW_VRSN                               0x0007
#define maH20VALVE                              0x0008
#define maFRESHAIR                              0x0009
#define maSCHEDULE_ON_R                         0x000A
#define maSCHEDULE_OFF_R                        0x000B
#define maMIN_FREQUENCY                         0x000C
#define maMAX_FREQUENCY                         0x000E
#define maPID_CONST                             0x0010
#define maFLOW_SPAN                             0x0012
#define maDATE_VFD                              0x0013
#define maTIME_VFD                              0x0015
#define maSET_CO2                               0x0017
#define maSET_DUCTPRESS                         0x0018
#define maSETWATER_DELTA_T                      0x001A
#define maSETMAX_FLOWRATE                       0x001C
#define maSET_TOTAL_VAV                         0x001E
#define maFlowmeter_Select                      0x001F
#define maVFD_Type                              0x0020
#define maDuct_Press_Span                       0x0021
#define maPresConst                             0x0022
#define maInletThres                            0x0023
#define maUVAHUCont                             0x0024
#define maUVCurThreshold                        0x0025
#define maUVCTSel                               0x0026
#define maUVLmtSwchCont                         0x0027
#define maUVSchStatus                           0x0028
#define maUVSchONTime                           0x0029
#define maUVSchOFFTime                          0x002A
#define maUVStatus                              0x002B
#define maFacSpeed                              0x002C
#define maFacSchStatus                          0x002D
#define maFacSchONTime                          0x002E
#define maFacSchOffTime                         0x002F
#define maBTUSelection                          0x0030
#define maMinWaterValvePos                      0x0031
#define maMinFlowRate                           0x0032
#define maWaterPressSpan                        0x0033
#define maFlowMeterSpan                         0x0034
  
#define ma1relay0                               0x0035
#define ma1relay1                               0x0036
#define ma1relay2                               0x0037
#define ma1relay3                               0x0038

#define ma2relay0                               0x0039
#define ma2relay1                               0x003A
#define ma2relay2                               0x003B
#define ma2relay3                               0x003C
  
  
#define ma3relay0                               0x003D
#define ma3relay1                               0x003E
#define ma3relay2                               0x003F
#define ma3relay3                               0x0040
  



/******************************** VAV HODING (R/W)*******************************/
#define VAVHOFFSET                            0x9C41
#define maSETTEMP                               0x0000
#define maDAMPOSITION                           0x0002
   
typedef struct{
  uint16_t transacId;           //MBAP Header
  uint16_t protocolId;
  uint16_t length;
  uint8_t unitId;
  uint8_t funcCode;              //PDU
  uint8_t data[MB_DATA_LEN];
}modbus_packet;   

struct FEEDVFD{
uint8_t FillInput1[VFDINPUTLEN];
uint8_t FillHoldR[VFDHOLDLEN];
//uint8_t FillHoldW[VFDHOLDLEN];
};
struct FEEDVAV{
uint8_t FillCoil[COILLENGTH];
uint8_t FillInput[VAVINPUTLENGTH];
uint8_t FillVAVHodWr[VAVHOLDLENGTH];
};
struct PRESSURE{
  uint16_t pressure1;
  uint16_t pressure2;
  uint16_t pressure3;
  uint16_t pressure4;
  uint16_t pressure5;
};
void clear_data();
  //extern struct PRESSURE pump_prs;
extern uint8_t stat_to_send[MB_DATA_LEN];
modbus_packet mb_parsepacket(uint8_t data[], uint16_t len);
uint8_t * mb_processpacketVAV(modbus_packet packet, uint16_t *sz);
uint8_t * mb_processpacketVFD(modbus_packet packet, uint16_t *sz);
void ExceptionSend(uint8_t ExpType, uint8_t FuncCode);
int SetVFD_Coil(uint16_t address, uint8_t ValueToWrite);
void GetVFDLastAddHoldRead(uint16_t endaddress);
void GetVFDPosHoldReadVFD(uint16_t Address,uint16_t endaddress);
void GetVFDHoldDataRead(void);
void GetVFDLastAddInputRead(uint16_t endaddress);
void GetVFDPosInputRead(uint16_t Address,uint16_t lastadd);
void GetVFDInputData(void);
int SetVFDHoldingParams(uint16_t Address,uint32_t ValueToWrite);
void GetVAVCoilPos(uint16_t Address,uint16_t endaddress);
void GetVAVCoilData(uint8_t index);
void GetVAVLastAddInputRead(uint16_t endaddress);
void GetVAVPosInputRead(uint16_t Address,uint16_t lastadd);
void GetVAVInputData(uint8_t index);
void GetVAVLastAddressHoldingRead(uint16_t endaddress);
void GetVAVPosHoldingRead(uint16_t Address,uint16_t lastadd);
void GetVavHoldReadData(uint8_t index);
void fvReadModbusTcp(struct PRESSURE *revModbusData);
#ifdef __cplusplus
}
#endif

#endif /*__MODBUSPROCESS_H*/