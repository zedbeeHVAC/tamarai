#include "ModbusProcess.h"
#include "Menu.h"
#include "Drive_Communication.h"
#include "Algo.h"
#include "Auto_Algo.h"
#include "Menu.h"
#include "cjson_packet.h"
#include "actuator-vfd.h"
#include "vav-data.h"
#include "vav-communication.h"
#include "Timer.h"
#include "FRAM.h"
#include "Keypad_Driver.h"
#include "sensor-card-data.h"
#include "ADC.h"
#include "UV.h"
#include "modbus_master.h"
uint8_t stat_to_send[MB_DATA_LEN] = {0},CoilDataReadfor_comiss[2]={0},CoilDataRead[4] = {0};
uint8_t startpos=0,endpos=0;
static struct stRS485Params ModbusRS485Params;
static struct TstMenuItems ModbusMenuItems;
static struct sensor_card_data_tag sensor_card_data_rcv;
static struct vav_data_tag g_vav_data_tcp[MAXVAV];
static struct FEEDVFD MODBUSFVFD;
static struct FEEDVAV MODBUSFVAV[4];
static RTC_TimeTypeDef ModbusTime;
static RTC_DateTypeDef ModbusDate;
struct PRESSURE pump_prs;
uint16_t Address=0,endaddress=0,lastadd=0,TotalReg=0;


struct stRelayModbusdata relaymodbus[5];
//struct stChillerholdregister chiller[4];
//static struct sensor_card_data_tag modbus_sensor_card_data;

void fvReadModbusTcp(struct PRESSURE *revModbusData)
{
  memcpy(revModbusData,&pump_prs,sizeof(struct PRESSURE));
}
modbus_packet mb_parsepacket(uint8_t data[], uint16_t len) 
{
  uint8_t i =0, j=0;
  modbus_packet packet;

  packet.transacId = (uint16_t)((data[i]<<8) + data[i+1]); //i = 0,1 //Start MBAP Header
  i+=2;
  packet.protocolId = (uint16_t)((data[i]<<8) + data[i+1]);// i = 2,3
  i+=2;
  packet.length = (uint16_t)((data[i]<<8) + data[i+1]);    // i = 4,5
  i+=2;
  packet.unitId = data[i];                              //i=6 //End of MBAP Header
  
  i++;
  packet.funcCode = data[i];                            //i=7
  i++;
  
  for(j=0; j<MB_DATA_LEN; j++)                          //i= 7--55
    packet.data[j] = data[i++];
  return packet;
}

uint8_t * mb_processpacketVFD(modbus_packet packet, uint16_t *sz)
{
  fvRS485ParamsRead(&ModbusRS485Params);
  fvMenuItemsRead(&ModbusMenuItems);
  uint8_t j=0,index=5;
  uint8_t TempCoil=0,TempCoilfor_comiss=0;
  uint8_t length=0,CoilWritten=0,TotalByteToFollow=0,CoilValue=0,WriteC_Value=0;
  uint16_t NextAddress=0,TotalCoils=0,NextAddressfor_comiss=0;
  uint32_t valuetowriteH=0;
  int ReturnReg=0,WrittenRegH=0;
  
  
  stat_to_send[j++] = (uint8_t)((packet.transacId&0xFF00)>>8);        //0          
  stat_to_send[j++] = (uint8_t)(packet.transacId&0xFF);               //1
  
  stat_to_send[j++] = (uint8_t)((packet.protocolId&0xFF00)>>8);       //2      
  stat_to_send[j++] = (uint8_t)(packet.protocolId&0xFF);              //3
  
  stat_to_send[j++] = (uint8_t)((packet.length&0xFF00)>>8);           //4  
  stat_to_send[j++] = (uint8_t)(packet.length&0xFF);                  //5
  
  stat_to_send[j++] = packet.unitId;                                  //6
  stat_to_send[j++] = packet.funcCode;                                //7
  
  switch(packet.funcCode)
  { 
  case RDCOILSTAT:
    TotalCoils =(uint16_t)(((packet.data[2])<<8)+(packet.data[3])); 
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1])); 
    /* If data addreee and requested coil's value is within the range then send
    *the values of the quired params otherwise send the exceptions
    */
    if((Address>=maT_P_CONTROL)&&(Address<=maH2O_Act_DIR))  
    {
      if(TotalCoils==2)
      {
        stat_to_send[j++]= MAXByteCoil ; // no of bytes to follow (we have only 3 coils so 1 byte is enough) 
        
        CoilDataReadfor_comiss[0]=ModbusMenuItems.u16PressTempSel;
        CoilDataReadfor_comiss[1]=ModbusMenuItems.u16ActuatorDir;
        
        /*  fill the data based on start address and no of coils quired */
        for(int i=0; i<TotalCoils; i++)
        {    
          NextAddressfor_comiss =  i;
          TempCoilfor_comiss =(uint8_t)(TempCoilfor_comiss |
                                        ( CoilDataReadfor_comiss[NextAddressfor_comiss]<<NextAddressfor_comiss));
        }
        
        stat_to_send[j++]= TempCoilfor_comiss;
      }
      
      else
      {
        /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
        ExceptionSend(ILLDATAADDRESS,packet.funcCode);
      }
    }
    else if((TotalCoils>=1)&&(TotalCoils<=MAXCOILVFD))
    {
      if((Address>=maVFDSTATUS)&&(Address<=maH2O_Act_DIR))
      {
        
        /* send no of bytes to follow (we have only 3 coils and one coil will
        *  occupay only one bit .So, 1 byte is enough for all params.)
        */ 
        stat_to_send[j++]= MAXByteCoil ; // no of bytes to follow (we have only 3 coils so 1 byte is enough) 
        
        CoilDataRead[0] = ModbusRS485Params.u8VfdState;
        CoilDataRead[1] = ModbusMenuItems.u16ScheduleOnOff;
        CoilDataRead[2] = ModbusMenuItems.u16PressTempSel;
        CoilDataRead[3] = ModbusMenuItems.u16ActuatorDir;
        /*  fill the data based on start address and no of coils quired */
        for(int i=0; i<TotalCoils; i++)
        {
          NextAddress = Address + i;
          TempCoil =(uint8_t)(TempCoil |( CoilDataRead[NextAddress]<<NextAddress));
        }
        
        stat_to_send[j++]= TempCoil;
      }
      
      else
      {
        /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
        ExceptionSend(ILLDATAADDRESS,packet.funcCode);
      }
    }
    
    break;
    
  case WRONECOIL:
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1])); 
    TotalCoils  =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
    TotalByteToFollow = packet.data[4];        
    CoilValue = packet.data[5];
    WriteC_Value=0;
    if((Address>=maVFDSTATUS)&&(Address<=maH2O_Act_DIR))
    {
      if((TotalCoils>=1)&&(TotalCoils<=MAXCOILVFD))
      {
        if(TotalByteToFollow==1 )
        {
          for(int i=0;i< TotalCoils; i++)
          { 
            if(TotalCoils==1)
            {
              WriteC_Value = CoilValue;
              CoilWritten = CoilWritten + SetVFD_Coil(Address,WriteC_Value);
            }else{
              NextAddress = Address + i ;
              WriteC_Value = CoilValue%2;
              CoilValue = CoilValue/2; 
              CoilWritten = CoilWritten + SetVFD_Coil(NextAddress,WriteC_Value);
            }
            
          }
          stat_to_send[j++]= packet.data[0];
          stat_to_send[j++]= packet.data[1];
          stat_to_send[j++]= 0; 
          stat_to_send[j++]= CoilWritten;
        }
        else
        {
          /*  If no of coils requested is wrong then send "ILLDATAVALUE" exception*/
          ExceptionSend(ILLDATAVALUE,packet.funcCode);
        }
      }
      else
      {
        /*  If no of coils requested is wrong then send "ILLDATAVALUE" exception*/
        ExceptionSend(ILLDATAVALUE,packet.funcCode);
      }
    }
    else 
    {
      /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
      ExceptionSend(ILLDATAADDRESS,packet.funcCode);
    }
    
    break;
    
  case RDHLDNGREG:      
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));    
    stat_to_send[j++] = TotalReg *2;        
      if((Address>=maFREQUENCY)&&(Address<= ma3relay3))
      {
        if((TotalReg>=1)&&(TotalReg <= MAXHREGVFD))
        {
          endaddress = Address + TotalReg ;
          GetVFDLastAddHoldRead(endaddress);  //getting last address 
          GetVFDPosHoldReadVFD(Address,lastadd);
          GetVFDHoldDataRead();
          for(int i = startpos; i <= endpos; i++)
          {
            stat_to_send[j++]=  MODBUSFVFD.FillHoldR[i];   
          }
          
        }else
        {
          /*  If no of coils requested is wrong then send "ILLDATAVALUE" exception*/
          ExceptionSend(ILLDATAVALUE,packet.funcCode);
        }
      }else
      {
        /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
        ExceptionSend(ILLDATAADDRESS,packet.funcCode);
      }
    break;
  case RDINREG:
    
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));        
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
    stat_to_send[j++] =((uint8_t) TotalReg)*2 ; //8 
    if((Address>=maRESERVED )&&(Address<=magpio14 ))
    {
      if((TotalReg>=1)&&(TotalReg <= MAXINREGVFD))
      {
        endaddress = Address + TotalReg ;
        GetVFDLastAddInputRead(endaddress);  
        GetVFDPosInputRead(Address,lastadd); 
        GetVFDInputData();
        for(int i = startpos; i <= endpos; i++)
        {
          stat_to_send[j++]=  MODBUSFVFD.FillInput1[i];
        }
      }
      else
      {
        /*  If no of coils requested is wrong then send "ILLDATAVALUE" exception*/
        ExceptionSend(ILLDATAVALUE,packet.funcCode);
      }
    }
    else
    {
      /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
      ExceptionSend(ILLDATAADDRESS,packet.funcCode);
    }
    
    break;
  case WRHOLDING_REG:
    
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
        
    if((Address >= maFREQUENCY)&&(Address<=ma3relay3))
    {
      if(TotalReg > 0)
      {
        while(TotalReg > 0)                  
        {
          if((Address == maFREQUENCY)||(Address== maSET_TEMP_VFD)||(Address==maMIN_FREQUENCY)||
             (Address == maMAX_FREQUENCY)||(Address == maPID_CONST)||(Address==maDATE_VFD)||
               (Address==maTIME_VFD)||(Address==maSET_DUCTPRESS)||(Address==maSETWATER_DELTA_T)||
                 (Address==maSETMAX_FLOWRATE))
          {
            valuetowriteH = (uint32_t)(((packet.data[index]<<24)&0xFF000000)+
                                       ((packet.data[index+1]<<16)&0x00FF0000)+((packet.data[index+2]<<8)
                                                                                &0x0000FF00)+(packet.data[index+3]&0x000000FF));
            index= index+4;
            ReturnReg = SetVFDHoldingParams(Address,valuetowriteH);
            Address=Address+2;
            TotalReg=TotalReg-2;
          }else
          {
            valuetowriteH = (uint32_t) ((packet.data[index] << 8)&0xFF00)+
              ((packet.data[index+1]&0x00FF));
            index=index+2;
            ReturnReg = SetVFDHoldingParams(Address,valuetowriteH);
            Address=Address+1;
            TotalReg=TotalReg-1;
          }
          WrittenRegH=ReturnReg+WrittenRegH;      
        }
        
        stat_to_send[j++]= packet.data[0];  
        stat_to_send[j++]= packet.data[1];
        stat_to_send[j++]= (uint8_t)((WrittenRegH&0xFF00)>>8);
        stat_to_send[j++]= (uint8_t)(WrittenRegH&0x00FF);
      }else
      {
        /*  If no of coils requested is wrong then send "ILLDATAVALUE" exception*/
        ExceptionSend(ILLDATAVALUE,packet.funcCode);
      }   // address   
    }else
    {
      /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
      ExceptionSend(ILLDATAADDRESS,packet.funcCode);
    }
    
    break;
  default :
    ExceptionSend(ILLEGALFUNC,packet.funcCode);
    break;
  }
  
  startpos=0;
  endpos =0;
  Address=0;
  TotalReg=0;
  endaddress=0;
  lastadd=0;
  
  *sz=(uint16_t)MB_DATA_LEN;
  return stat_to_send;
}
uint8_t * mb_processpacketVAV(modbus_packet packet, uint16_t *sz)
{
  uint8_t j=0,index=0;
  stat_to_send[j++] = (uint8_t)((packet.transacId&0xFF00)>>8);        //0          
  stat_to_send[j++] = (uint8_t)(packet.transacId&0xFF);               //1
  
  stat_to_send[j++] = (uint8_t)((packet.protocolId&0xFF00)>>8);       //2      
  stat_to_send[j++] = (uint8_t)(packet.protocolId&0xFF);              //3
  
  stat_to_send[j++] = (uint8_t)((packet.length&0xFF00)>>8);           //4  
  stat_to_send[j++] = (uint8_t)(packet.length&0xFF);                  //5
  
  stat_to_send[j++] = packet.unitId;                                  //6
  stat_to_send[j++] = packet.funcCode;                                //7
  index = (packet.unitId);
  vav_data_read(index,&g_vav_data_tcp[index]);
  switch(packet.funcCode)
  {
  case RDCOILSTAT:          // func code 01
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
    
    stat_to_send[j++] =((uint8_t) TotalReg) ;                          //8
    endaddress = (Address + TotalReg )-1;
    GetVAVCoilPos(Address,endaddress); 
    
    if(Address==0)
    {
      GetVAVCoilData(index);  
      for(int i = startpos; i <= endpos; i++)
      {
        stat_to_send[j++]=MODBUSFVAV[index].FillCoil[i];      
      }
    }
    else
    {
      /*  If data address of first coil is wrong then send "ILLDATAADDRESS" exception*/
      ExceptionSend(ILLDATAADDRESS,packet.funcCode);
    }
    break;
  case WRONECOIL:
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1])); 
    
    g_vav_data_tcp[index].on_off_status = packet.data[5];
    g_vav_data_tcp[index].id = index;
    g_vav_data_tcp[index].identify |= 1;
    vav_data_func_code_write(index,2);
    vav_data_write(index,&g_vav_data_tcp[index]);
    //transmitenable();
    
    stat_to_send[j++]= packet.data[0];
    stat_to_send[j++]= packet.data[1];
    stat_to_send[j++]= packet.data[2];
    stat_to_send[j]= packet.data[3];
    break;
    
  case RDINREG:              // func code 04
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
    stat_to_send[j++] =((uint8_t) TotalReg)*2 ;                          //8
    
    if((Address>=address1) && (Address<=address59))
    {   
      if((TotalReg>=1)&&(TotalReg <= MAXINPUTVAVREG))
      {
        endaddress = Address + TotalReg ;
        GetVAVLastAddInputRead(endaddress);
        GetVAVPosInputRead(Address,lastadd); 
        GetVAVInputData(index);
        for(int i = startpos; i <= endpos; i++)
        {
          stat_to_send[j++]=MODBUSFVAV[index].FillInput[i];      
        }
      }
      else
      {
        ExceptionSend(ILLDATAVALUE,packet.funcCode);
      } 
    }
    else
    {
      ExceptionSend(ILLDATAADDRESS,packet.funcCode);
    }
    
    break;  
    
  case RDHLDNGREG:
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
    if((Address>=maSETTEMP) && (Address<=maDAMPOSITION))
    {
      if((TotalReg>=1)&&(TotalReg <= MAXHREGVAV))
      {
        endaddress = Address + TotalReg ;
        GetVAVLastAddressHoldingRead(endaddress);  //getting last address 
        GetVAVPosHoldingRead(Address,lastadd);
        GetVavHoldReadData(index);
        
        stat_to_send[j++] =((uint8_t) (TotalReg*2)) ; 
        for(int i = startpos; i <= endpos; i++)
        {
           stat_to_send[j++]= MODBUSFVAV[index].FillVAVHodWr[i];   
        }

      }
      else
      {
        ExceptionSend(ILLDATAVALUE,packet.funcCode);
      }
    }
    else
    {
      ExceptionSend(ILLDATAADDRESS,packet.funcCode);
    }
    break;
  case WRHOLDING_REG:
    Address = (uint16_t)(((packet.data[0])<<8)+(packet.data[1]));
    TotalReg =(uint16_t)(((packet.data[2])<<8)+(packet.data[3]));
    if(Address == maSETTEMP)
    {
    //  uint32_t set_value= (uint32_t)(((packet.data[5]<<24)&0xFF000000)+
      //                   ((packet.data[6]<<16)&0x00FF0000)+((packet.data[7]<<8)
        //                             &0x0000FF00)+(packet.data[8]&0x000000FF));
      uint16_t set_value=       ((packet.data[5] << 8)&0xFF00)+((packet.data[6]&0x00FF));
      float setpoint=((float)set_value/100);
      fvModbusWriteMaxFrq_pump(index,setpoint);

    /*  if(set_value > BMS_MIN_TEMP && set_value < MAX_SET_TEMP )
      {
        g_vav_data_tcp[index].set_temp =(uint16_t)set_value;
        g_vav_data_tcp[index].id = index;
        g_vav_data_tcp[index].identify |= (1 << 1);//settemp
        vav_data_func_code_write(index,2);
        vav_data_write(index,&g_vav_data_tcp[index]);
      }*/
      stat_to_send[j++]= packet.data[0];  
      stat_to_send[j++]= packet.data[1];
      stat_to_send[j++]= 0;  //no of reg to written hi
      stat_to_send[j]= 2;    // no of reg to written lo
    }
    else if(Address == maDAMPOSITION)
    {
   //  SetBMSFailFlag(1);
     // g_vav_data_tcp[index].damper_pos =(((packet.data[5])<<8)+(packet.data[6]));
   //  uint8_t start= (((packet.data[5])<<8)+(packet.data[6]));  
     
     uint16_t start=       ((packet.data[5] << 8)&0xFF00)+((packet.data[6]&0x00FF));
     
     if(start>0){start=65280;}
        else {start=0;}       
        fvModbusWriteVfd_pump(index,14,start);
    
     // g_vav_data_tcp[index].id = index;
     // g_vav_data_tcp[index].identify |= (1 << 2);//pos
     // vav_data_func_code_write(index,2);
      //vav_data_write(index,&g_vav_data_tcp[index]);
      stat_to_send[j++]= packet.data[0];  
      stat_to_send[j++]= packet.data[1];
      stat_to_send[j++]= 0;  //no of reg to written hi
      stat_to_send[j]= 1;    // no of reg to written lo
    }
    break;
    
  default:
    break;
  }
  startpos=0;
  endpos =0;
  Address=0;
  TotalReg=0;
  endaddress=0;
  lastadd=0;
  
  *sz=(uint16_t)MB_DATA_LEN;   
  return stat_to_send;
}
void GetVavHoldReadData(uint8_t index)
{
  struct stChillerholdregister chiller;
  struct stChillerCoildata comp;
    uint8_t damp_pos=0;
    fvReadChillerHoldParams(&chiller,index);
    fvReadChillerCoilParams(&comp,index);
 // vav_data_read(index,&g_vav_data_tcp[index]);
  uint32_t stempint =(uint32_t)(chiller.ChilledWaterSetpoint);//(g_vav_data_tcp[index].set_temp); 
  damp_pos = (uint8_t)comp.coil8;
  
  MODBUSFVAV[index].FillVAVHodWr[0] = (uint8_t)((stempint & 0xFF000000)>>24); 
  MODBUSFVAV[index].FillVAVHodWr[1] = (uint8_t)((stempint & 0x00FF0000)>>16);
  MODBUSFVAV[index].FillVAVHodWr[2] = (uint8_t)((stempint & 0x0000FF00)>>8);
  MODBUSFVAV[index].FillVAVHodWr[3] = (uint8_t)(stempint & 0x000000FF); 
      
  MODBUSFVAV[index].FillVAVHodWr[4] =0;
  MODBUSFVAV[index].FillVAVHodWr[5]=damp_pos;
}
void GetVAVPosHoldingRead(uint16_t Address,uint16_t lastadd)
{
  switch(Address)
  {
    case maSETTEMP:
    startpos=0;
    break;
    case maDAMPOSITION:
    startpos=4;
    break;
    
    default:
    break;
  }
  switch(lastadd)
  {
    case maSETTEMP:
    endpos=3;
    break;
    case maDAMPOSITION:
    endpos=5;
    
    break;
    default:
    break;
  }
}
void GetVAVLastAddressHoldingRead(uint16_t endaddress)
{
  if(TotalReg!=MAXHREGVAV)
  {
    if(endaddress<=maDAMPOSITION)
    {
      switch(endaddress)
      {
      case maSETTEMP:
        lastadd=maSETTEMP;
        break;
      case maDAMPOSITION:
        lastadd=maSETTEMP;
        break;
      default:
        break;  
      }
    }
    else
    {
      lastadd = maDAMPOSITION;
    }
  }
  else
  {
    lastadd = maDAMPOSITION;
  }
}
void GetVAVInputData(uint8_t index)
{
  struct stChillerholdregister chiller;
  struct stChillerCoildata comp;
  uint8_t x=0;
 // vav_data_read(index,&g_vav_data_tcp[index]);
  HAL_RTC_GetDate(&hrtc,&ModbusDate,RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc,&ModbusTime,RTC_FORMAT_BIN);

  for(int i=0; i<VAVINPUTLENGTH;i++)
    MODBUSFVAV[index].FillInput[i]=0x00;
  
 
   fvReadChillerHoldParams(&chiller,index);
   fvReadChillerCoilParams(&comp,index);
   
     uint32_t v1 = chiller.ActiveCoolHeatSetpointTemp;
   MODBUSFVAV[index].FillInput[x++] =(uint8_t)((v1 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++] =(uint8_t)((v1 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++] =(uint8_t)((v1 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++] =(uint8_t)(v1 & 0x000000FF);         //3
           
   uint32_t v2	= ((float)chiller. ActiveCurrentLimitSetpoint/100) ;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v2 & 0xFF000000)>>24);    //4
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v2 & 0x00FF0000)>>16);    //5
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v2 & 0x0000FF00)>>8);     //6
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v2 & 0x000000FF);         //7
           

uint32_t v3	=chiller. EvapRefrigerantPressure_Ckt1;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v3 & 0xFF000000)>>24);    //8
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v3 & 0x00FF0000)>>16);    //9
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v3 & 0x0000FF00)>>8);     //10
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v3 & 0x000000FF);         //11
           
uint32_t v4	=chiller. EvapRefrigerantPressure_Ckt2 ;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v4 & 0xFF000000)>>24);    //12
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v4 & 0x00FF0000)>>16);    //13
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v4 & 0x0000FF00)>>8);     //14
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v4& 0x000000FF);         //15
           
uint32_t v5	= chiller. CondenserRefrigerantPressure_Ckt1; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v5 & 0xFF000000)>>24);    //16
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v5 & 0x00FF0000)>>16);    //17
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v5 & 0x0000FF00)>>8);     //18
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v5 & 0x000000FF);         //19
           
uint32_t v6 = chiller. CondenserRefrigerantPressure_Ckt2 ; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v6 & 0xFF000000)>>24);    //20
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v6 & 0x00FF0000)>>16);    //21
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v6 & 0x0000FF00)>>8);     //22
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v6 & 0x000000FF);         //23
           
uint32_t v7	= ((float)chiller. RunTime_Comp2B/100) ;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v7 & 0xFF000000)>>24);    //24
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v7 & 0x00FF0000)>>16);    //25
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v7 & 0x0000FF00)>>8);     //26
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v7 & 0x000000FF);         //27
           
uint32_t v8	= ((float)chiller. Starts_Comp1A)/100;         
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v8 & 0xFF000000)>>24);    //28
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v8 & 0x00FF0000)>>16);    //29
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v8 & 0x0000FF00)>>8);     //30
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v8 & 0x000000FF);         //31
           
uint32_t v9 = ((float)chiller. Starts_Comp1B)/100;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v9 & 0xFF000000)>>24);    //32
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v9 & 0x00FF0000)>>16);    //33
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v9 & 0x0000FF00)>>8);     //34
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v9 & 0x000000FF);         //35
           
uint32_t v10	= ((float)chiller. Starts_Comp2A)/100 ;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v10 & 0xFF000000)>>24);    //36
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v10 & 0x00FF0000)>>16);    //37
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v10 & 0x0000FF00)>>8);     //38
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v10 & 0x000000FF);         //39
           
uint32_t v11	= ((float)chiller. Starts_Comp2B)/100 ;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v11 & 0xFF000000)>>24);    //40
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v11 & 0x00FF0000)>>16);    //41
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v11 & 0x0000FF00)>>8);     //42
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v11 & 0x000000FF);         //43
           
uint32_t v12	= ((float)chiller. RunTime_Comp1A/100) ;    
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v12 & 0xFF000000)>>24);    //44
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v12 & 0x00FF0000)>>16);    //45
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v12 & 0x0000FF00)>>8);     //46
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v12& 0x000000FF);         //47
           
uint32_t v13	= ((float)chiller. RunTime_Comp1B/100) ; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v13 & 0xFF000000)>>24);    //48
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v13 & 0x00FF0000)>>16);    //49
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v13 & 0x0000FF00)>>8);     //50
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v13 & 0x000000FF);         //51
           
uint32_t v14	= ((float)chiller. RunTime_Comp2A/100);  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v14 & 0xFF000000)>>24);    //52
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v14 & 0x00FF0000)>>16);    //53
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v14 & 0x0000FF00)>>8);     //54
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v14 & 0x000000FF);         //55
           
uint32_t v15	=chiller. Line2Current_inAmps_Comp2A;    
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v15 & 0xFF000000)>>24);    //56
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v15 & 0x00FF0000)>>16);    //57
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v15 & 0x0000FF00)>>8);     //58
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v15 & 0x000000FF);         //59
           
uint32_t v16	= chiller. EvapLeavingWaterTemp ;
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v16 & 0xFF000000)>>24);    //60
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v16 & 0x00FF0000)>>16);    //61
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v16 & 0x0000FF00)>>8);     //62
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v16 & 0x000000FF);         //63
           
uint32_t v17	=chiller. HighSideOilPressure_Comp1A; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v17 & 0xFF000000)>>24);    //64
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v17 & 0x00FF0000)>>16);    //65
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v17 & 0x0000FF00)>>8);     //66
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v17 & 0x000000FF);         //67
           
uint32_t v18	=chiller. HighSideOilPressure_Comp1B;
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v18 & 0xFF000000)>>24);    //68
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v18 & 0x00FF0000)>>16);    //69
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v18 & 0x0000FF00)>>8);     //70
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v18 & 0x000000FF);         //71
           
uint32_t v19	=chiller. HighSideOilPressure_Comp2A;
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v19 & 0xFF000000)>>24);    //72
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v19 & 0x00FF0000)>>16);    //73
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v19 & 0x0000FF00)>>8);     //74
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v19 & 0x000000FF);         //75
           
uint32_t v20	=chiller. HighSideOilPressure_Comp2B;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v20 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v20 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v20 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v20 & 0x000000FF);         //3
           
uint32_t v21	=chiller. OilTemp_Comp1A;     
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v21 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v21 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v21 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v21 & 0x000000FF);         //3
uint32_t v22	=chiller. OilTemp_Comp1B;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v22 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v22 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v22 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v22 & 0x000000FF);         //3
uint32_t	v23	=	chiller. OilTemp_Comp2A;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v23 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v23 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v23 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v23 & 0x000000FF);         //3
uint32_t	v24	=	chiller. EvapEnteringWaterTemp;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v24 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v24 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v24 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v24 & 0x000000FF);         //3
uint32_t	v25	=	chiller. OilTemp_Comp2B;    
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v25 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v25 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v25 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v25 & 0x000000FF);         //3
uint32_t	v26	=	chiller. PhaseABVoltage_Comp1A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v26 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v26 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v26 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v26 & 0x000000FF);         //3
uint32_t	v27	=	chiller. Line1Current_inAmps_Comp1A;
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v27 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v27 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v27 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v27 & 0x000000FF);         //3
uint32_t	v28	=	chiller. Line2Current_inAmps_Comp1A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v28 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v28 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v28 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v28 & 0x000000FF);         //3
uint32_t	v29	=	chiller. Line3Current_inAmps_Comp1A; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v29 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v29 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v29 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v29 & 0x000000FF);         //3
uint32_t	v30	=	chiller. Line1Current_inAmps_Comp1B;
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v30 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v30 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v30 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v30 & 0x000000FF);         //3
uint32_t	v31	=	chiller. Line2Current_inAmps_Comp1B; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v31 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v31 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v31 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v31 & 0x000000FF);         //3
uint32_t	v32	=	chiller. Line3Current_inAmps_Comp1B; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v32 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v32 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v32 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v32 & 0x000000FF);         //3
uint32_t	v33	=	chiller. Line1Current_inAmps_Comp2A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v33 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v33 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v33 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v33 & 0x000000FF);         //3
uint32_t	v34	=	chiller. OutdoorAirTemp ;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v34 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v34 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v34 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v34 & 0x000000FF);         //3
uint32_t	v35	=	chiller. Line3Current_inAmps_Comp2A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v35 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v35 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v35 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v35 & 0x000000FF);         //3
uint32_t	v36	=	chiller. Line1Current_inAmps_Comp2B;    
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v36 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v36 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v36 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v36 & 0x000000FF);         //3
uint32_t	v37	=	chiller. Line2Current_inAmps_Comp2B; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v37 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v37 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v37 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v37 & 0x000000FF);         //3
uint32_t	v38	=	chiller. Line1Current_RLA_Comp1A;    
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v38 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v38 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v38 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v38 & 0x000000FF);         //3
uint32_t	v39	=	chiller. Line2Current_RLA_Comp1A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v39 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v39 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v39 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v39 & 0x000000FF);         //3
uint32_t	v40	=	chiller. Line3Current_RLA_Comp1A; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v40 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v40 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v40 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v40 & 0x000000FF);         //3
uint32_t	v41	=	chiller. Line1Current_RLA_Comp1B; 
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v41 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v41 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v41 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v41 & 0x000000FF);         //3
uint32_t	v42	=	chiller. Line2Current_RLA_Comp1B;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v42 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v42 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v42 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v42 & 0x000000FF);         //3
uint32_t	v43	=	chiller. Line3Current_RLA_Comp1B;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v43 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v43 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v43 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v43 & 0x000000FF);         //3
uint32_t	v44	=	chiller. Line1Current_RLA_Comp2A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v44 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v44 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v44 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v44 & 0x000000FF);         //3
uint32_t	v45	=	chiller. Line2Current_RLA_Comp2A;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v45 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v45 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v45 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v45 & 0x000000FF);         //3
uint32_t	v46	=	chiller. Line3Current_RLA_Comp2A;    
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v46 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v46 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v46 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v46 & 0x000000FF);         //3
uint32_t	v47	=	chiller. Line1Current_RLA_Comp2B;   
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v47 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v47 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v47 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v47 & 0x000000FF);         //3
uint32_t	v48	=	chiller. Line2Current_RLA_Comp2B;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v48 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v48 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v48 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v48 & 0x000000FF);         //3
uint32_t	v49	=chiller. Line3Current_RLA_Comp2B;  
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v49 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v49 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v49 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v49 & 0x000000FF);         //3
uint32_t	v50	=chiller. ChilledWaterSetpoint;       
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v50 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v50 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v50 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v50 & 0x000000FF);         //3
uint32_t	v51	=chiller. Line3Current_inAmps_Comp2B;
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v51 & 0xFF000000)>>24);    //0
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v51 & 0x00FF0000)>>16);    //1
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)((v51 & 0x0000FF00)>>8);     //2
   MODBUSFVAV[index].FillInput[x++]=(uint8_t)(v51 & 0x000000FF);         //3
//-----------------on off status

   uint16_t v52 = comp.coil1;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v52&0xFF00)>>8);       //4
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v52&0x00FF);           //5
    uint16_t v53 = comp.coil2;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v53&0xFF00)>>8);       //4
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v53&0x00FF);           //5
    uint16_t v54 = comp.coil3;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v54&0xFF00)>>8);       //4
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v54&0x00FF);           //5
    uint16_t v55 = comp.coil4;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v55&0xFF00)>>8);       //4
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v55&0x00FF); 
  
    uint16_t v56 = comp.coil5;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v56&0xFF00)>>8);       //37
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v56&0x00FF);           //38
  
    uint16_t v57 = comp.coil6;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v57&0xFF00)>>8);       //39
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v57&0x00FF);           //40
  
     uint16_t v58 = comp.coil7;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v58&0xFF00)>>8);       //39
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)(v58&0x00FF);           //40
       uint16_t v59 = comp.coil8;
  MODBUSFVAV[index].FillInput[x++] = (uint8_t)((v59&0xFF00)>>8);       //41
  MODBUSFVAV[index].FillInput[x] = (uint8_t)(v59&0x00FF);           //42
}
void GetVAVPosInputRead(uint16_t Address,uint16_t lastadd)
{
  switch(Address)
  {
  case address1:
    startpos=0;
    break;
  case address2:
    startpos=4;
    break;
  case address3:
    startpos= 8;
    break;
  case address4:
    startpos=12;
    break;
  case address5:
    startpos=16;
    break;
  case address6:
    startpos=20;
    break;
  case address7:
    startpos=24;
    break;
  
    case address8:
    startpos=28;
    break;
  case address9:
    startpos=32;
    break;
  case address10:
    startpos=36;
    break;
  case address11:
    startpos=40;
    break;
  case address12:
    startpos=44;
    break;
  case address13:
    startpos=48;
    break;
  case address14:
    startpos=52;
    break;
    
  
    case address15:
    startpos=56;
    break;
  case address16:
    startpos=60;
    break;
  case address17:
    startpos=64;
    break;
  case address18:
    startpos=68;
    break;
  case address19:
    startpos=72;
    break;
  case address20:
    startpos=76;
    break;
  case address21:
    startpos=80;
    break;
  
     case address22:
    startpos=84;
    break;
  case address23:
    startpos=88;
    break;
  case address24:
    startpos= 92;
    break;
  case address25:
    startpos=96;
    break;
  case address26:
    startpos=100;
    break;
  case address27:
    startpos=104;
    break;
  case address28:
    startpos=108;
    break;
  
    case address29:
    startpos=112;
    break;
  case address30:
    startpos=116;
    break;
  case address31:
    startpos=120;
    break;
  case address32:
    startpos=124;
    break;
  case address33:
    startpos=128;
    break;
  case address34:
    startpos=132;
    break;
  case address35:
    startpos=136;
    break;
    
  
    case address36:
    startpos=140;
    break;
  case address37:
    startpos=144;
    break;
  case address38:
    startpos=148;
    break;
  case address39:
    startpos=152;
    break;
  case address40:
    startpos=156;
    break;
   
  case address41:
    startpos=160;
    break;
  case address42:
    startpos=164;
    break;
  case address43:
    startpos=168;
    break;
  case address44:
    startpos=172;
    break;
  case address45:
    startpos=176;
    break;
  case address46:
    startpos=180;
    break;
    
  
    case address47:
    startpos=184;
    break;
  case address48:
    startpos=188;
    break;
  case address49:
    startpos=192;
    break;
  case address50:
    startpos=196;
    break;
  case address51:
    startpos=200;
    break;
  
      
    case address52:
    startpos=204;
    break;
  case address53:
    startpos=206;
    break;
  case address54:
    startpos=208;
    break;
  case address55:
    startpos=210;
    break;
  case address56:
    startpos=212;
    break;
 case address57:
    startpos=214;
    break;
  case address58:
    startpos=216;
    break;
  case address59:
    startpos=218;
    break;
  default:
    break;
  }
  
  switch(lastadd)
  {
case address1:
    endpos=3;
    break;
  case address2:
    endpos=7;
    break;
  case address3:
    endpos= 11;
    break;
  case address4:
    endpos=15;
    break;
  case address5:
    endpos=19;
    break;
  case address6:
    endpos=23;
    break;
  case address7:
    endpos=27;
    break;
  
    case address8:
    endpos=31;
    break;
  case address9:
    endpos=35;
    break;
  case address10:
    endpos=39;
    break;
  case address11:
    endpos=43;
    break;
  case address12:
    endpos=47;
    break;
  case address13:
    endpos=51;
    break;
  case address14:
    endpos=55;
    break;
    
  
    case address15:
    endpos=59;
    break;
  case address16:
    endpos=63;
    break;
  case address17:
    endpos=67;
    break;
  case address18:
    endpos=71;
    break;
  case address19:
    endpos=75;
    break;
  case address20:
    endpos=79;
    break;
  case address21:
    endpos=83;
    break;
  
     case address22:
    endpos=87;
    break;
  case address23:
    endpos=91;
    break;
  case address24:
    endpos= 95;
    break;
  case address25:
    endpos=99;
    break;
  case address26:
    endpos=103;
    break;
  case address27:
    endpos=107;
    break;
  case address28:
    endpos=111;
    break;
  
    case address29:
    endpos=115;
    break;
  case address30:
    endpos=119;
    break;
  case address31:
    endpos=123;
    break;
  case address32:
    endpos=127;
    break;
  case address33:
    endpos=131;
    break;
  case address34:
    endpos=135;
    break;
  case address35:
    endpos=139;
    break;
    
  
    case address36:
    endpos=143;
    break;
  case address37:
    endpos=147;
    break;
  case address38:
    endpos=151;
    break;
  case address39:
    endpos=155;
    break;
  case address40:
    endpos=159;
    break;
   case address41:
    endpos=163;
    break;
  case address42:
    endpos=167;
    break;
  case address43:
    endpos= 171;
    break;
  case address44:
    endpos=175;
    break;
  case address45:
    endpos=179;
    break;
  case address46:
    endpos=183;
    break;
  case address47:
    endpos=187;
    break;
  
    case address48:
    endpos=191;
    break;
  case address49:
    endpos=195;
    break;
  case address50:
    endpos=199;
    break;
  case address51:
    endpos=203;
    break;
  case address52:
    endpos=205;
    break;
  case address53:
    endpos=207;
    break;
  case address54:
    endpos=209;
    break;
    
  
    case address55:
    endpos=211;
    break;
  case address56:
    endpos=213;
    break;
  case address57:
    endpos=215;
    break;
  case address58:
    endpos=217;
    break;
 
    
  case address59:
    endpos=219;  
    break;
  default:
    break;      
  }
}
void GetVAVLastAddInputRead(uint16_t endaddress)
{
   if(TotalReg != MAXINPUTVAVREG)
   {
     if(endaddress<= address59)
     {
       switch(endaddress)
       {
      case address1:
    lastadd=address1;
    break;
  case address2:
    lastadd=address1;
    break;
  case address3:
    lastadd= address2;
    break;
  case address4:
    lastadd=address3;
    break;
  case address5:
    lastadd=address4;
    break;
  case address6:
    lastadd=address5;
    break;
  case address7:
    lastadd=address6;
    break;
  
    case address8:
    lastadd=address7;
    break;
  case address9:
    lastadd=address8;
    break;
  case address10:
    lastadd=address9;
    break;
  case address11:
    lastadd=address10;
    break;
  case address12:
    lastadd=address11;
    break;
  case address13:
    lastadd=address12;
    break;
  case address14:
    lastadd=address13;
    break;
    
  
    case address15:
    lastadd=address14;
    break;
  case address16:
    lastadd=address15;
    break;
  case address17:
    lastadd=address16;
    break;
  case address18:
    lastadd=address17;
    break;
  case address19:
    lastadd=address18;
    break;
  case address20:
    lastadd=address19;
    break;
  case address21:
    lastadd=address20;
    break;
  
     case address22:
    lastadd=address21;
    break;
  case address23:
    lastadd=address22;
    break;
  case address24:
    lastadd= address23;
    break;
  case address25:
    lastadd=address24;
    break;
  case address26:
    lastadd=address25;
    break;
  case address27:
    lastadd=address26;
    break;
  case address28:
    lastadd=address27;
    break;
  
    case address29:
    lastadd=address28;
    break;
  case address30:
    lastadd=address29;
    break;
  case address31:
    lastadd=address30;
    break;
  case address32:
    lastadd=address31;
    break;
  case address33:
    lastadd=address32;
    break;
  case address34:
    lastadd=address33;
    break;
  case address35:
    lastadd=address34;
    break;
    
  
    case address36:
    lastadd=address35;
    break;
  case address37:
    lastadd=address36;
    break;
  case address38:
    lastadd=address37;
    break;
  case address39:
    lastadd=address38;
    break;
  case address40:
    lastadd=address39;
    break;
  case address41:
    lastadd=address40;
    break;
  case address42:
    lastadd=address41;
    break;
    case address43:
    lastadd=address42;
    break;
    case address44:
    lastadd=address43;
    break;
    case address45:
    lastadd=address44;
    break;
    case address46:
    lastadd=address45;
    break;
   case address47:
    lastadd=address46;
    break;
     case address48:
    lastadd=address47;
    break;
     case address49:
    lastadd=address48;
    break;
     case address50:
    lastadd=address49;
    break;
     case address51:
    lastadd=address50;
    break;
     case address52:
    lastadd=address51;
    break;
     case address53:
    lastadd=address52;
    break;
     case address54:
    lastadd=address53;
    break;
     case address55:
    lastadd=address54;
    break;
     case address56:
    lastadd=address55;
    break;
     case address57:
    lastadd=address56;
    break;
     case address58:
    lastadd=address57;
    break;
     case address59:
    lastadd=address58;
    break;
  
         
         
         
       default:
         break;
       }
     }
     else
     {
       lastadd = address59;
     }
   }
   else
   {
     lastadd = address59;
   } 
}
void GetVAVCoilData(uint8_t index)
{
  vav_data_read(index,&g_vav_data_tcp[index]);
  MODBUSFVAV[index].FillCoil[0] = (uint8_t)((g_vav_data_tcp[index].on_off_status)&&0x00FF);
  
}
void GetVAVCoilPos(uint16_t Address,uint16_t endaddress)
{
  switch(Address)
  {
  case maVAVSTATUS:
    startpos=0;
    break;
  default:
    break;
  }
  switch(endaddress)
  {
  case maVAVSTATUS:
    endpos=0;
    break;
  default:
    break;      
  }
}
int SetVFD_Coil(uint16_t address, uint8_t ValueToWrite)
{  
  fvRS485ParamsRead(&ModbusRS485Params);
  fvMenuItemsRead(&ModbusMenuItems);
  uint8_t ReturnWCoil=0;
  if((ValueToWrite==0)||(ValueToWrite==1))
  {
    ReturnWCoil=1;
    
    switch(address)
    {
    case maVFDSTATUS:
     
      fvSetBmsStatus(ValueToWrite,(void*)(&ModbusMenuItems));
     /* volatile uint8_t u8VfdStatus = fu8GetMachineStartState((void*)(&(ModbusRS485Params)),(void*)(&ModbusMenuItems));            
      if(u8VfdStatus != ValueToWrite)
      {
	if( ModbusMenuItems.u16UV_AHU_Control || Get_UVOverCurrent_Error())
        {
            Set_UVOverCurrent_Clear();
            uint8_t UV_State = u8VfdStatus;
            fvUVSetBmsStatus(UV_State);
            UV_State = fu8UVGetMachineStartState((void*)(&ModbusMenuItems));
            fvSetUVStatus(UV_State);
        }*/

        //fvSendVFDCommand(VFD_ONOFF);
         if(ValueToWrite == VFD_OFF)
      {
        GPIOE->ODR &= ~GPIO_PIN_1;
        //GPIOE->ODR &= ~GPIO_PIN_14;
        //ModbusRS485Params.u8SystemState = SYS_STOP_STATE;
        //float freq = 0;
       // fvFreqApply(freq);
        ModbusRS485Params.u8VfdState = ValueToWrite;
        fvRS485ParamsWrite(&ModbusRS485Params);
        ReturnWCoil=1;
      }
      else if(ValueToWrite == VFD_ON)
      {
        GPIOE->ODR |= GPIO_PIN_1;
        //GPIOC->ODR |= GPIO_PIN_7;
        //GPIOE->ODR |= GPIO_PIN_14;
       // ModbusRS485Params.u8SystemState = SYS_RUNNING_STATE;
        //float freq = (((float)ModbusMenuItems.u16MaxFreq)/10);
        //fvFreqApply(freq);
        ModbusRS485Params.u8VfdState = ValueToWrite;
        fvRS485ParamsWrite(&ModbusRS485Params);
        ReturnWCoil=1;
      }
        
      
      break;
    case maSCHEDULE_ENABLE:      
      ModbusMenuItems.u16ScheduleOnOff = ValueToWrite;
      fvMenuItemsWrite(&ModbusMenuItems);
      fvMenustore();
      
      break;
    case maT_P_CONTROL:
      ModbusMenuItems.u16PressTempSel = ValueToWrite;
      fvMenuItemsWrite(&ModbusMenuItems);
      fvMenustore();
      
      break;
    case maH2O_Act_DIR:
      ModbusMenuItems.u16ActuatorDir = ValueToWrite;
      fvMenuItemsWrite(&ModbusMenuItems);
      fvMenustore();
      break;
    default:
      ExceptionSend(ILLDATAADDRESS,WRONECOIL);
      ReturnWCoil=0;
      break;  
    }
  }
  return ReturnWCoil;  
}
void GetVFDLastAddHoldRead(uint16_t endaddress)
{
   if(TotalReg!=MAXHREGVFD)
  {
    if(endaddress<=ma3relay3)
    {
      switch(endaddress)
      {
      case maFREQUENCY:
        lastadd=maFREQUENCY;
        break;
      case maDIAGPACK:
        lastadd = maFREQUENCY;
        break;
      case maMODESELEST:
        lastadd = maDIAGPACK;
        break;
      case maSET_TEMP_VFD:
        lastadd = maMODESELEST;
        break;
      case maTFTP_RQST:
        lastadd = maSET_TEMP_VFD;
        break;
      case maSW_VRSN:
        lastadd = maTFTP_RQST;
        break;
      case maH20VALVE:
        lastadd = maSW_VRSN;
        break;
      case maFRESHAIR:
        lastadd = maH20VALVE;
        break;
      case maSCHEDULE_ON_R:
        lastadd = maFRESHAIR;
        break;
      case maSCHEDULE_OFF_R:
        lastadd = maSCHEDULE_ON_R;
        break;
      case maMIN_FREQUENCY:
        lastadd = maSCHEDULE_OFF_R;
        break;
      case maMAX_FREQUENCY:
        lastadd = maMIN_FREQUENCY;
        break;
      case maPID_CONST:
        lastadd = maMAX_FREQUENCY;
        break;
      case maFLOW_SPAN:
        lastadd = maPID_CONST;
        break;
      case maDATE_VFD:
        lastadd = maFLOW_SPAN;
        break;
      case maTIME_VFD:
        lastadd = maDATE_VFD;
        break;
      case maSET_CO2:
        lastadd = maTIME_VFD;
        break;
        
      case maSET_DUCTPRESS:
        lastadd = maSET_CO2;
        break;
      case maSETWATER_DELTA_T:
        lastadd = maSET_DUCTPRESS;
        break;
      case maSETMAX_FLOWRATE:
        lastadd = maSETWATER_DELTA_T;
        break;
      case maSET_TOTAL_VAV:
        lastadd = maSETMAX_FLOWRATE;
        break;
      case maFlowmeter_Select:
        lastadd = maSET_TOTAL_VAV;
        break; 
      case maVFD_Type:
        lastadd = maFlowmeter_Select;
        break;
      case maDuct_Press_Span:
        lastadd = maVFD_Type;
        break;
        
      case maPresConst:
        lastadd = maDuct_Press_Span;
        break;
       case maInletThres:
        lastadd = maPresConst;
        break;
      case maUVAHUCont:
        lastadd = maInletThres;
        break; 
      case maUVCurThreshold:
        lastadd = maUVAHUCont;
        break;
      case maUVCTSel:
        lastadd = maUVCurThreshold;
        break;
      case maUVLmtSwchCont:
        lastadd = maUVCTSel;
        break;
      case maUVSchStatus:
        lastadd = maUVLmtSwchCont;
        break;
      case maUVSchONTime:
        lastadd = maUVSchStatus;
        break; 
      case maUVSchOFFTime:
        lastadd = maUVSchONTime;
        break;
      case maUVStatus:
        lastadd = maUVSchOFFTime;
        break;
      case maFacSpeed:
        lastadd = maUVStatus;
        break;
      case maFacSchStatus:
        lastadd = maFacSpeed;
        break; 
      case maFacSchONTime:
        lastadd = maFacSchStatus;
        break;
      case maFacSchOffTime:
        lastadd = maFacSchONTime;
        break; 
         case maBTUSelection:
        lastadd = maFacSchOffTime;
        break;
      case maMinWaterValvePos:
        lastadd = maBTUSelection;
        break;
      case maMinFlowRate:
        lastadd = maMinWaterValvePos;
        break; 
      case maWaterPressSpan:
        lastadd = maMinFlowRate;
        break;
      case maFlowMeterSpan:
        lastadd = maWaterPressSpan;
        break; 
        
      case ma1relay0:
        lastadd = maFlowMeterSpan;
        break;
      case ma1relay1:
        lastadd = ma1relay0;
        break;
      case ma1relay2:
        lastadd = ma1relay1;
        break; 
      case ma1relay3:
        lastadd = ma1relay2;
        break;
    
      case ma2relay0:
        lastadd = ma1relay3;
        break;
      case ma2relay1:
        lastadd = ma2relay0;
        break;
      case ma2relay2:
        lastadd = ma2relay1;
        break; 
      case ma2relay3:
        lastadd = ma2relay2;
        break;
        
        case ma3relay0:
        lastadd = ma2relay3;
        break;
      case ma3relay1:
        lastadd = ma3relay0;
        break;
      case ma3relay2:
        lastadd = ma3relay1;
        break; 
      case ma3relay3:
        lastadd = ma3relay2;
        break; 
      default:
        break;
      }
    }
    else
    {
      lastadd = ma3relay3;
    }
    
  }else
  {
    lastadd = ma3relay3;
  }
}
void GetVFDPosHoldReadVFD(uint16_t Address,uint16_t endaddress)
{
  switch(Address)
  {
  case maFREQUENCY:
    startpos=0;
    break;
  case maDIAGPACK:
    startpos=4;
    break;
  case maMODESELEST:
    startpos=6;
    break;
  case maSET_TEMP_VFD:
    startpos=8;
    break;      
  case maTFTP_RQST:
    startpos=12;
    break;
  case maSW_VRSN:
    startpos=14;
    break;
  case maH20VALVE:
    startpos=16;
    break;
  case maFRESHAIR:
    startpos=18;
    break;
  case maSCHEDULE_ON_R:
    startpos=20;
    break;
  case maSCHEDULE_OFF_R:
    startpos=22;
    break;
  case maMIN_FREQUENCY:
    startpos=24;
    break;
  case maMAX_FREQUENCY:
    startpos=28;
    break;
  case maPID_CONST:
    startpos=32;
    break;
  case maFLOW_SPAN:
    startpos=36;
    break;
  case maDATE_VFD:
    startpos=38;
    break;
  case maTIME_VFD:
    startpos=42;
    break;
  case maSET_CO2:
    startpos=46;
    break;
  case maSET_DUCTPRESS:
    startpos = 48;
    break;
  case maSETWATER_DELTA_T:
    startpos = 52;
    break;
  case maSETMAX_FLOWRATE:
    startpos = 56;
    break;  
  case maSET_TOTAL_VAV:
    startpos = 60;
    break;
  case maFlowmeter_Select:
    startpos = 62;
    break; 
  case maVFD_Type:
    startpos = 64;
    break;
  case maDuct_Press_Span:
    startpos = 66;
    break;
    
 case maPresConst:
    startpos = 68;
    break;
   case maInletThres:
    startpos=70;
    break;
  case maUVAHUCont:
    startpos=72;
    break;
  case maUVCurThreshold:
    startpos = 74;
    break;
  case maUVCTSel:
    startpos = 76;
    break;
  case maUVLmtSwchCont:
    startpos = 78;
    break;  
  case maUVSchStatus:
    startpos = 80;
    break;
  case maUVSchONTime:
    startpos = 82;
    break; 
  case maUVSchOFFTime:
    startpos = 84;
    break;
  case maUVStatus:
    startpos = 86;
    break;
  case maFacSpeed:
    startpos = 88;
    break;
  case maFacSchStatus:
    startpos = 90;
    break; 
  case maFacSchONTime:
    startpos = 92;
    break;
  case maFacSchOffTime:
    startpos = 94;
    break;
  case maBTUSelection:
    startpos = 96;
    break;
  case maMinWaterValvePos:
    startpos = 98;
    break;
  case maMinFlowRate:
    startpos = 100;
    break; 
  case maWaterPressSpan:
    startpos = 102;
    break;
  case maFlowMeterSpan:
    startpos = 104;
    break;
  case ma1relay0:
    startpos = 106;
    break;
  case ma1relay1:
    startpos = 108;
    break;
  case ma1relay2:
    startpos = 110;
    break; 
  case ma1relay3:
    startpos = 112;
    break;
 
   case ma2relay0:
    startpos = 114;
    break;
  case ma2relay1:
    startpos = 116;
    break;
  case ma2relay2:
    startpos = 118;
    break; 
  case ma2relay3:
    startpos = 120;
    break;
 
 case ma3relay0:
    startpos = 122;
    break;
  case ma3relay1:
    startpos = 124;
    break;
  case ma3relay2:
    startpos = 126;
    break;    
  case ma3relay3:
    startpos = 128;
    break;
  default:
    break;
  }
  
  switch(endaddress)
  {
  case maFREQUENCY:
    endpos=3;
    break;
  case maDIAGPACK:
    endpos=5;
    break;
  case maMODESELEST:
    endpos=7;
    break;
  case maSET_TEMP_VFD:
    endpos=11;
    break;     
  case maTFTP_RQST:
    endpos=13;
    break;
  case maSW_VRSN:
    endpos=15;
    break;
  case maH20VALVE:
    endpos=17;
    break;
  case maFRESHAIR:
    endpos=19;
    break;
  case maSCHEDULE_ON_R:
    endpos=21;
    break;
  case maSCHEDULE_OFF_R:
    endpos=23;
    break;
  case maMIN_FREQUENCY:
    endpos=27;
    break;
  case maMAX_FREQUENCY:
    endpos=31;
    break;
  case maPID_CONST:
    endpos=35;
    break;
  case maFLOW_SPAN:
    endpos=37;
    break;
  case maDATE_VFD:
    endpos=41;
    break;
  case maTIME_VFD:
    endpos=45;
    break;
  case maSET_CO2:
    endpos=47;
    break;
    
  case maSET_DUCTPRESS:
    endpos = 51;
    break;
  case maSETWATER_DELTA_T:
    endpos = 55;
    break;
  case maSETMAX_FLOWRATE:
    endpos = 59;
    break;
  case maSET_TOTAL_VAV:
    endpos = 61;
    break;
  case maFlowmeter_Select:
    endpos = 63;
    break;
  case maVFD_Type:
    endpos = 65;
    break;
  case maDuct_Press_Span:
    endpos = 67;
    break;
    
   case maPresConst:
    endpos = 69;
    break;
   case maInletThres:
    endpos=71;
    break;
  case maUVAHUCont:
    endpos=73;
    break;
  case maUVCurThreshold:
    endpos = 75;
    break;
  case maUVCTSel:
    endpos = 77;
    break;
  case maUVLmtSwchCont:
    endpos = 79;
    break;  
  case maUVSchStatus:
    endpos = 81;
    break;
  case maUVSchONTime:
    endpos = 83;
    break; 
  case maUVSchOFFTime:
    endpos = 85;
    break;
  case maUVStatus:
    endpos = 87;
    break;
  case maFacSpeed:
    endpos = 89;
    break;
  case maFacSchStatus:
    endpos = 91;
    break; 
  case maFacSchONTime:
    endpos = 93;
    break;
  case maFacSchOffTime:
    endpos = 95;
    break;
  case maBTUSelection:
    endpos = 97;
    break;
  case maMinWaterValvePos:
    endpos = 99;
    break;
  case maMinFlowRate:
    endpos = 101;
    break; 
  case maWaterPressSpan:
    endpos = 103;
    break;
  case maFlowMeterSpan:
    endpos = 105;
    break;
    
   case ma1relay0:
    endpos = 107;
    break;
  case ma1relay1:
    endpos = 109;
    break;
  case ma1relay2:
    endpos = 111;
    break; 
  case ma1relay3:
    endpos = 113;
    break;
    
  case ma2relay0:
    endpos = 115;
    break;
  case ma2relay1:
    endpos = 117;
    break;
  case ma2relay2:
    endpos = 119;
    break; 
  case ma2relay3:
    endpos = 121;
    break;
  
    case ma3relay0:
    endpos = 123;
    break;
  case ma3relay1:
    endpos = 125;
    break;
  case ma3relay2:
    endpos = 127;
    break; 
  case ma3relay3:
    endpos =129;
    break;
  default:
    break;      
  }
}
void GetVFDHoldDataRead(void)
{
   fvMenuItemsRead(&ModbusMenuItems);
   HAL_RTC_GetDate(&hrtc,&ModbusDate,RTC_FORMAT_BIN);
   HAL_RTC_GetTime(&hrtc,&ModbusTime,RTC_FORMAT_BIN);
   uint8_t x=0;
   /* for DIAG,software download,and software version have seprate query will come */
   for(int i=0;i<VFDHOLDLEN; i++)
      MODBUSFVFD.FillHoldR[i]=0;
   
   uint32_t frequency_int = (uint32_t)(get_VFDFreq()*100);             
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((frequency_int & 0xFF000000)>>24);    //0
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((frequency_int & 0x00FF0000)>>16);    //1
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((frequency_int & 0x0000FF00)>>8);     //2
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(frequency_int & 0x000000FF);          //3
   /** DIAG **/
   MODBUSFVFD.FillHoldR[x++]= 0; //4
   MODBUSFVFD.FillHoldR[x++]= 0; //5
   /******Mode*******/
   MODBUSFVFD.FillHoldR[x++]= 0;                                     //6
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)ModbusMenuItems.u16AutoMan; //7
   /** INPUT1- Set Temprature **/
   uint32_t VFDSetTmp_int = (uint32_t)(ModbusMenuItems.u16SetTemp*10);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetTmp_int & 0xFF000000)>>24);    //8
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetTmp_int & 0x00FF0000)>>16);    //9
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetTmp_int & 0x0000FF00)>>8);     //10
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(VFDSetTmp_int & 0x000000FF);          //11
   /** INPUT3 - Software Download**/
   MODBUSFVFD.FillHoldR[x++]= 0; //12
   MODBUSFVFD.FillHoldR[x++]= 0; //13
   /** INPUT4 - Software Version **/
   MODBUSFVFD.FillHoldR[x++]= 0;  //14
   MODBUSFVFD.FillHoldR[x++]= 0;  //15
   /** INPUT2 - H2O Valve **/        
   int H2oValve =  (int)fu8GetWaterValvePercent();
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((H2oValve & 0xFF00)>>8); //16
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(H2oValve & 0x00FF);        //17
   /** INPUT5 - Fresh Air Valve **/  
   sensor_card_data_read(&sensor_card_data_rcv);
   int freshairpos = sensor_card_data_rcv.fa_valve_position;
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((freshairpos & 0xFF00)>>8); //18
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(freshairpos & 0x00FF);        //19
   /** INPUT6 - Schedule On Time **/
    uint16_t u16Temp = ModbusMenuItems.u16ScheduleONTime;
     MODBUSFVFD.FillHoldR[x++]= (uint8_t)((u16Temp & 0xFF00)>>8);          //20
     MODBUSFVFD.FillHoldR[x++]= (uint8_t)(u16Temp & 0x00FF);       //21
   /** INPUT7 - Schedule Off Time**/
     u16Temp  = ModbusMenuItems.u16ScheduleOFFTime;
     MODBUSFVFD.FillHoldR[x++]= (uint8_t)((u16Temp & 0xFF00)>>8);        //22
     MODBUSFVFD.FillHoldR[x++]= (uint8_t)(u16Temp & 0x00FF);        //23
   /** INPUT8 - min frequency **/
   uint32_t min_freq_int =(uint32_t)(ModbusMenuItems.u16MinFreq*10);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((min_freq_int & 0xFF000000)>>24);    //24
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((min_freq_int & 0x00FF0000)>>16);    //25
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((min_freq_int & 0x0000FF00)>>8);     //26
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(min_freq_int & 0x000000FF);          //27
   /** INPUT9 - max frequency**/
   uint32_t max_freq_int =(uint32_t) (ModbusMenuItems.u16MaxFreq*10);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((max_freq_int & 0xFF000000)>>24);    //28
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((max_freq_int & 0x00FF0000)>>16);    //29
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((max_freq_int & 0x0000FF00)>>8);     //30
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(max_freq_int & 0x000000FF);          //31
   /** INPUT10 - pid constant**/
   uint32_t PID_CONST_INT = (uint32_t)(ModbusMenuItems.u16PIDConst);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((PID_CONST_INT & 0xFF000000)>>24);    //32
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((PID_CONST_INT & 0x00FF0000)>>16);    //33
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((PID_CONST_INT & 0x0000FF00)>>8);     //34
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(PID_CONST_INT & 0x000000FF);          //35
   
   /******************************* Flow Span******************************/
   if(ModbusMenuItems.u16FlowmeterType == MA_4_20)
   {
      MODBUSFVFD.FillHoldR[x++]= 0;  //36
      MODBUSFVFD.FillHoldR[x++]= ModbusMenuItems.u16Span1;//37
   }
   else if(ModbusMenuItems.u16FlowmeterType == V_0_10)
   {
      MODBUSFVFD.FillHoldR[x++]= 0;  //36
      MODBUSFVFD.FillHoldR[x++]= ModbusMenuItems.u16Span2;//37
   }
   else
   {
      MODBUSFVFD.FillHoldR[x++]= 0;  //36
      MODBUSFVFD.FillHoldR[x++]= ModbusMenuItems.u16Span3;//37
   }
   /************  DATE ********************/
   uint16_t Year = 2000+ModbusDate.Year;
   MODBUSFVFD.FillHoldR[x++] = ModbusDate.Date;//38 // sending day
   MODBUSFVFD.FillHoldR[x++] = ModbusDate.Month; // 39 // sending month
   MODBUSFVFD.FillHoldR[x++] = (uint8_t)(((Year)&0xFF00)>>8);//40 sending year
   MODBUSFVFD.FillHoldR[x++] = (uint8_t)((Year)&0x00FF);   //41
   /********************Time *****************/
   MODBUSFVFD.FillHoldR[x++] = (uint8_t)((ModbusTime.Hours)&0x000000FF);  //42
   MODBUSFVFD.FillHoldR[x++] = (uint8_t)((ModbusTime.Minutes)&0x000000FF);  //43
   MODBUSFVFD.FillHoldR[x++] = (uint8_t)(ModbusTime.Seconds)&(0x000000FF);  //44
   MODBUSFVFD.FillHoldR[x++] = 0;                                         //45
   /******************* Set CO2***********************************/
   int co2 = ModbusMenuItems.u16SetCO2;
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((co2 & 0xFF00)>>8); //46
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(co2 & 0x00FF);        //47
   /******************* Set Duct Pressure ***********************************/
   uint32_t VFDSetDuctPress = (uint32_t)(ModbusMenuItems.u16DuctSetPressure);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetDuctPress & 0xFF000000)>>24);    //48
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetDuctPress & 0x00FF0000)>>16);    //49
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetDuctPress & 0x0000FF00)>>8);     //50
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(VFDSetDuctPress & 0x000000FF);          //51
   /******************* Set Water Delta T ***********************************/   
   uint32_t VFDSetWaterDeltaT = (uint32_t)(ModbusMenuItems.u16WaterDeltaT);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetWaterDeltaT & 0xFF000000)>>24);    //52
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetWaterDeltaT & 0x00FF0000)>>16);    //53
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetWaterDeltaT & 0x0000FF00)>>8);     //54
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(VFDSetWaterDeltaT & 0x000000FF);          //55
   /******************* Set Max water Flow Rate ***********************************/   
   uint32_t VFDSetMaxFlow = (uint32_t)(ModbusMenuItems.u16MaxFlowrate);
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetMaxFlow & 0xFF000000)>>24);    //56
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetMaxFlow & 0x00FF0000)>>16);    //57
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((VFDSetMaxFlow & 0x0000FF00)>>8);     //58
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)(VFDSetMaxFlow & 0x000000FF);          //59
   /******************* Number of VAV ***********************************/
   MODBUSFVFD.FillHoldR[x++]=0;   //60
   MODBUSFVFD.FillHoldR[x++]=ModbusMenuItems.u16VavNumber;//61
   /******************************* FLOW meter type***********************************/    
   MODBUSFVFD.FillHoldR[x++]=0;// 62
   MODBUSFVFD.FillHoldR[x++]=ModbusMenuItems.u16FlowmeterType;// 63
   /****************************** VFD type*********************************/   
   MODBUSFVFD.FillHoldR[x++]=0;// 64
   MODBUSFVFD.FillHoldR[x++]= ModbusMenuItems.u16VfdType;//  65
   /*************************Duct pressure span*********************************/
   MODBUSFVFD.FillHoldR[x++]=0;             //66
   MODBUSFVFD.FillHoldR[x++]=ModbusMenuItems.u16DuctPressureSpan;//
   
      /************************Pressure constant *********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //68
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //69
   /************************inlet threshold *********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16Inlet_threshold & 0xFF00)>>8); //70
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16Inlet_threshold & 0x00FF);        //71
    /************************UV AHU control *********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16UV_AHU_Control & 0xFF00)>>8); //72
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16UV_AHU_Control & 0x00FF);        //73
   /************************UV current threshold *********************************/
   
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //74
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //75
      /************************UV cT selection *********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //76
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //77
    /************************UVlimit switch control *********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //78
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //79
    /************************UVschedule status*********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //80
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //81
       /************************UVschedule ON Time*********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //82
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //83
       /***********************UVschedule OFF Time*********************************/
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((ModbusMenuItems.u16PressureConstant & 0xFF00)>>8); //84
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(ModbusMenuItems.u16PressureConstant & 0x00FF);        //85
          /************************UV status*********************************/
   
  uint16_t uvstatus=(uint16_t)fvGetUVStatus();
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((uvstatus & 0xFF00)>>8); //86
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(uvstatus & 0x00FF);        //87
   /************************Fac Speed*********************************/
    uint16_t facSpeed=0;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((facSpeed & 0xFF00)>>8); //88
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(facSpeed & 0x00FF);        //89
    /************************Fac Schedule Status*********************************/
    uint16_t facSchStatus=0;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((facSchStatus & 0xFF00)>>8); //90
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(facSchStatus & 0x00FF);        //91
    /************************Fac Schedule ONTime*********************************/
    uint16_t facSchONTime=0;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((facSchONTime & 0xFF00)>>8); //92
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(facSchONTime & 0x00FF);        //93
    /************************Fac Schedule OFFTime*********************************/
   uint16_t facSchOFFTime=0;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((facSchOFFTime & 0xFF00)>>8); //94
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(facSchOFFTime & 0x00FF);        //95
    /************************BTU Selection*********************************/
  uint16_t btuselect= ModbusMenuItems.u16BtuType;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((btuselect & 0xFF00)>>8); //96
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(btuselect & 0x00FF);        //97
   /************************water valve minimum position*********************************/
    uint16_t minvalvepos=ModbusMenuItems.u16MinValvePos*10;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((minvalvepos & 0xFF00)>>8); //98
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(minvalvepos & 0x00FF);        //99
    /************************Minimum flow rate*********************************/
    uint16_t minflowrate=ModbusMenuItems.u16MinFlowrate*10;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((minflowrate & 0xFF00)>>8); //100
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(minflowrate & 0x00FF);        //101
    /************************water pressure span*********************************/
    uint16_t waterpresspan=ModbusMenuItems.u16WaterBar*10;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((waterpresspan & 0xFF00)>>8); //102
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(waterpresspan & 0x00FF);        //103
    /************************Flowmeter span*********************************/
   uint16_t flowmeterSpan=ModbusMenuItems.u16MaxFlowSpan*10;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((flowmeterSpan & 0xFF00)>>8); //104
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(flowmeterSpan & 0x00FF);        //105
   
    /************************relay1*********************************/
fvReadRelayModbusParams(&relaymodbus[1],1);
   uint16_t maM1relay0=   relaymodbus[1].relay0;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM1relay0 & 0xFF00)>>8); //106
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM1relay0 & 0x00FF);        //107
   
    /************************relay-2*********************************/
   uint16_t maM1relay1=relaymodbus[1].relay1;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM1relay1 & 0xFF00)>>8); //108
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM1relay1 & 0x00FF);        //109
    /************************relay-3*********************************/
   uint16_t maM1relay2=relaymodbus[1].relay2;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM1relay2 & 0xFF00)>>8); //110
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM1relay2 & 0x00FF);        //111
    /************************relay-4*********************************/
    uint16_t maM1relay3=relaymodbus[1].relay3;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM1relay3 & 0xFF00)>>8); //112
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM1relay3 & 0x00FF);        //113
 
 
    /************************relay1*********************************/
fvReadRelayModbusParams(&relaymodbus[2],2);
   uint16_t maM2relay0=   relaymodbus[2].relay0;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM2relay0 & 0xFF00)>>8); //114
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM2relay0 & 0x00FF);        //115
   
    /************************relay-2*********************************/
   uint16_t maM2relay1=relaymodbus[2].relay1;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM2relay1 & 0xFF00)>>8); //116
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM2relay1 & 0x00FF);        //117
    /************************relay-3*********************************/
   uint16_t maM2relay2=relaymodbus[2].relay2;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM2relay2 & 0xFF00)>>8); //118
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM2relay2 & 0x00FF);        //119
    /************************relay-4*********************************/
    uint16_t maM2relay3=relaymodbus[2].relay3;
  MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM2relay3 & 0xFF00)>>8); //120
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM2relay3 & 0x00FF);        //121
   
   
    /************************relay1*********************************/
   fvReadRelayModbusParams(&relaymodbus[3],3);
   uint16_t maM3relay0=   relaymodbus[3].relay0;
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM3relay0 & 0xFF00)>>8); //122
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM3relay0 & 0x00FF);        //123
   
    /************************relay-2*********************************/
   uint16_t maM3relay1=relaymodbus[3].relay1;
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM3relay1 & 0xFF00)>>8); //124
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM3relay1 & 0x00FF);        //125
    /************************relay-3*********************************/
   uint16_t maM3relay2=relaymodbus[3].relay2;
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM3relay2 & 0xFF00)>>8); //126
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM3relay2 & 0x00FF);        //127
    /************************relay-4*********************************/
    uint16_t maM3relay3=relaymodbus[3].relay3;
   MODBUSFVFD.FillHoldR[x++]=(uint8_t)((maM3relay3 & 0xFF00)>>8); //128
   MODBUSFVFD.FillHoldR[x++]= (uint8_t)(maM3relay3 & 0x00FF);        //129
   
}
void GetVFDLastAddInputRead(uint16_t endaddress)
{
  if(TotalReg != MAXINREGVFD)
  {
    if(endaddress<= magpio14 )
    {
      switch(endaddress)
      {
      case maRESERVED:
        lastadd = maRESERVED;
        break;
      case maH20IN:
        lastadd = maRESERVED;
        break;
      case maH20OUT:
        lastadd = maH20IN;
        break;
      case maFLOWPERMIN:
        lastadd = maH20OUT;
        break;
      case maRETURNAIR:
        lastadd = maFLOWPERMIN;
        break;
      case maPOWER:
        lastadd = maRETURNAIR;
        break;
      case maBTUREADING:
        lastadd = maPOWER;
        break;
      case maC02LEVEL:
        lastadd = maBTUREADING;
        break;
      case maHUMIDITY:
        lastadd = maC02LEVEL;
        break;
      case maFILTERLEVEL:
        lastadd = maHUMIDITY;
        break;                  
      case maSUPPLYAIR:
        lastadd = maFILTERLEVEL;
        break;
      case  maDUCTPRESS:
        lastadd = maSUPPLYAIR;
        break;
      case maVOLTAGE:
        lastadd = maDUCTPRESS;
        break;
      case maCURRENT:
        lastadd = maVOLTAGE;
        break;
      case maRUNNING_HOURS:
        lastadd = maCURRENT;
        break;
      case maFIRE_SIGNAL:
        lastadd = maRUNNING_HOURS;
        break;
      case maVFD_ONOFF_Source:
        lastadd = maFIRE_SIGNAL;
        break;
        
      case maAQI:
        lastadd = maVFD_ONOFF_Source;
        break;
      case maPM1p0:
        lastadd = maAQI;
        break;
      case maPM10:
        lastadd = maPM1p0;
        break;
      case maPM2p5:
        lastadd = maPM10;
        break;
      case matvoc:
        lastadd = maPM2p5;
        break;
      case maCO:
        lastadd = matvoc;
        break;
      case maNH3:
        lastadd = maCO;
        break;
      case maNO2:
        lastadd = maNH3;
        break;
      case maAQC:
        lastadd = maNO2;
        break;
      case maLimitSwchStatus:
        lastadd = maAQC;
        break;
      case maUVRunHours:
        lastadd = maLimitSwchStatus;
        break;
      case maUVCurMeasured:
        lastadd = maUVRunHours;
        break;
       case maUVOffSource:
        lastadd = maUVCurMeasured;
        break;
      case maUVError:
        lastadd = maUVOffSource;
        break;
      case maFacOffSource:
        lastadd = maUVError;
        break;
       case maWaterPressure:
        lastadd = maFacOffSource;
        break; 
      
       case magpio1:
        lastadd = maWaterPressure;
        break; 
       case magpio2:
        lastadd = magpio1;
        break; 
       case magpio3:
        lastadd = magpio2;
        break; 
       case magpio4:
        lastadd = magpio3;
        break;
        case magpio5:
        lastadd = magpio4;
        break;
       case magpio6:
        lastadd = magpio5;
        break;
       case magpio7:
        lastadd = magpio6;
        break;
       case magpio8:
        lastadd = magpio7;
        break;
       case magpio9:
        lastadd = magpio8;
        break;
       case magpio10:
        lastadd = magpio9;
        break;
       case magpio11:
        lastadd = magpio10;
        break;
       case magpio12:
        lastadd = magpio11;
        break;
       case magpio13:
        lastadd = magpio12;
        break;
       case magpio14:
        lastadd = magpio13;
        break; 
      default:
        break;
      }
    }
    else
    {
      lastadd = magpio14;
    }
    
  }else
  {
    lastadd = magpio14;
  } 
}
void GetVFDPosInputRead(uint16_t Address,uint16_t lastadd)
{
  switch(Address)
  {
  case maRESERVED:
    startpos=0;
    break;
  case maH20IN:
    startpos=2;
    break;
  case maH20OUT:
    startpos=6;
    break;
  case maFLOWPERMIN:
    startpos=10;
    break;
  case maRETURNAIR:
    startpos=14;
    break;
  case maPOWER:
    startpos=18;
    break;
  case maBTUREADING:
    startpos=22;
    break;
  case maC02LEVEL:
    startpos=26;
    break;
  case maHUMIDITY:
    startpos=28;
    break;
  case maFILTERLEVEL:
    startpos=30;
    break;
  case maSUPPLYAIR:
    startpos = 32;
    break;
  case maDUCTPRESS:
    startpos = 36;
    break;
  case maVOLTAGE:
    startpos = 40;
    break;
  case maCURRENT:
    startpos = 44;
    break;
  case maRUNNING_HOURS:
    startpos = 48;
    break; 
   case maFIRE_SIGNAL:
    startpos = 52;
    break; 
  case maVFD_ONOFF_Source:
    startpos = 54;
    break;
    
    case maAQI:
    startpos = 56;
    break;
    case maPM1p0:
    startpos = 58;
    break;
    case maPM10:
    startpos = 60;
    break;
    case maPM2p5:
    startpos = 62;
    break;
    case matvoc:
    startpos = 64;
    break;
    case maCO:
    startpos = 66;
    break;
    case maNH3:
    startpos = 68;
    break;
    case maNO2:
    startpos = 70;
    break;
    case maAQC:
    startpos = 72;
    break;
    case maLimitSwchStatus:
    startpos = 74;
    break;
    case maUVRunHours:
    startpos = 76;
    break;
    case maUVCurMeasured:
    startpos = 80;
    break;
    case maUVOffSource:
    startpos = 82;
    break;
    case maUVError:
    startpos = 84;
    break;
    case maFacOffSource:
    startpos = 86;
    break;
    case maWaterPressure:
    startpos = 88;
    break;
    
    case magpio1:
    startpos = 90;
    break;
    case magpio2:
    startpos = 92;
    break;
    case magpio3:
    startpos = 94;
    break;
    case magpio4:
    startpos = 96;
    break;
    case magpio5:
    startpos = 98;
    break;
    case magpio6:
    startpos = 100;
    break;
    case magpio7:
    startpos = 102;
    break;
    case magpio8:
    startpos = 104;
    break;
    case magpio9:
    startpos = 106;
    break;
    case magpio10:
    startpos = 108;
    break;
    case magpio11:
    startpos = 110;
    break;
    case magpio12:
    startpos = 112;
    break;
    case magpio13:
    startpos = 114;
    break;
   
    case magpio14:
    startpos =116;
    break;
    
  default:
    break;
  }
  
  switch(lastadd)
  {
  case maRESERVED:
    endpos=1;
    break;
  case maH20IN:
    endpos=5;
    break;
  case maH20OUT:
    endpos=9;
    break;
  case maFLOWPERMIN:
    endpos=13;
    break;
  case maRETURNAIR:
    endpos=17;
    break;
  case maPOWER:
    endpos=21;
    break;
  case maBTUREADING:
    endpos=25;
    break;
  case maC02LEVEL:
    endpos=27;
    break;
  case maHUMIDITY:
    endpos=29;
    break;
  case maFILTERLEVEL:
    endpos=31;
    break;
  case maSUPPLYAIR:
    endpos = 35;
    break;
  case maDUCTPRESS:
    endpos = 39;
    break;
  case maVOLTAGE:
    endpos = 43;
    break; 
  case maCURRENT:
    endpos = 47;
    break; 
  case maRUNNING_HOURS:
    endpos = 51;
    break;
  case maFIRE_SIGNAL:
    endpos = 53;
    break;
   case maVFD_ONOFF_Source:
    endpos = 55;
    break;
    
  case maAQI:
    endpos = 57;
    break;
    case maPM1p0:
    endpos = 59;
    break;
    case maPM10:
    endpos = 61;
    break;
    case maPM2p5:
    endpos = 63;
    break;
    case matvoc:
    endpos = 65;
    break;
    case maCO:
    endpos = 67;
    break;
    case maNH3:
    endpos = 69;
    break;
    case maNO2:
    endpos = 71;
    break;
    case maAQC:
    endpos = 73;
    break;
    case maLimitSwchStatus:
    endpos = 75;
    break;
    case maUVRunHours:
    endpos = 79;
    break;
    case maUVCurMeasured:
    endpos = 81;
    case maUVOffSource:
    endpos = 83;
    break;
    case maUVError:
    endpos = 85;
    break;
    case maFacOffSource:
    endpos = 87;
    break;
    case maWaterPressure:
    endpos = 89;
    break;
  case magpio1:
    endpos = 91;
    break;
    case magpio2:
    endpos = 93;
    break;
    case magpio3:
    endpos = 95;
    break;
    case magpio4:
    endpos = 97;
    break;
    case magpio5:
    endpos = 99;
    break;
    case magpio6:
    endpos = 101;
    break;
    case magpio7:
    endpos = 103;
    break;
    case magpio8:
    endpos = 105;
    break;
    case magpio9:
    endpos = 107;
    break;
    case magpio10:
    endpos = 109;
    break;
    case magpio11:
    endpos = 111;
    break;
    case magpio12:
    endpos = 113;
    break;
    case magpio13:
    endpos = 115;
    break;
    case magpio14:
    endpos = 117 ;
    break;
    
  default:
    break;     
  }
}
void GetVFDInputData(void)
{  
   fvRS485ParamsRead(&ModbusRS485Params);
   uint8_t x=0;
   /************** Total VAV************************************/
   MODBUSFVFD.FillInput1[x++]=0;   // total vavs //0
   MODBUSFVFD.FillInput1[x++]=1;//0;//  future we can use any parameter here gstMenuItems.u16VavNumber;              //1     
   /********************* H2O in Temprature  ****************************/            
   uint32_t In_temp_int =(uint32_t)(u16Get_AvgWater_In());      
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((In_temp_int & 0xFF000000)>>24);//2
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((In_temp_int & 0x00FF0000)>>16);//3
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((In_temp_int & 0x0000FF00)>>8); //4
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(In_temp_int & 0x000000FF);      //5
   /******************* H2O Out temprature **********************/            
   uint32_t Out_temp_int = (uint32_t)(u16Get_AvgWater_Out());   
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Out_temp_int&0xFF000000)>>24); //6
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Out_temp_int&0x00FF0000)>>16); //7
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Out_temp_int&0x0000FF00)>>8);  //8
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(Out_temp_int&0x000000FF);       //9            
   /************************* H2O FLOW **************************************/  
   uint32_t Flow_min_int = (uint32_t)( u16Get_Avg_sup_Air());//(uint32_t)((u16Get_AvgFlowrate()*3600));  // converting into liters/hr        
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Flow_min_int&0xFF000000)>>24);  //10
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Flow_min_int&0x00FF0000)>>16);  //11
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Flow_min_int&0x0000FF00)>>8);   //12
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(Flow_min_int&0x000000FF);        //13
   /*********************** Return Air **********************************/           
   uint32_t Ret_temp_int = (uint32_t)(u16Get_AvgReturn_Air());  
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Ret_temp_int&0xFF000000)>>24);  //14
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Ret_temp_int&0x00FF0000)>>16);  //15
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Ret_temp_int&0x0000FF00)>>8);   //16
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(Ret_temp_int&0x000000FF);        //17    
   /************************* Power  ****************************************/  
   uint32_t Power_avg_int = (uint32_t)(u16Get_AvgPower());
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Power_avg_int&0xFF000000)>>24);  //18
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Power_avg_int&0x00FF0000)>>16);   //19
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((Power_avg_int&0x0000FF00)>>8);    //20
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(Power_avg_int&0x000000FF);       //21
   /*************************** BTU **************************************/            
   uint32_t BTU_New_int =(uint32_t)(GetBTU()/10);   
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((BTU_New_int&0xFF000000)>>24);  //22
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((BTU_New_int&0x00FF0000)>>16);  //23
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((BTU_New_int&0x0000FF00)>>8);   //24
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(BTU_New_int&0x000000FF);        //25       
   /************** CO2 Level ***********************************/   
   //struct sensor_card_data_tag sensor_card_data_rcv;
   sensor_card_data_read(&sensor_card_data_rcv);    
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.co2/*AmbCO2Level*/&0xFF00)>>8);    //26
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.co2/*AmbCO2Level*/&0x00FF);         //27
   /***************************** Humidity *******************************/     
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.humidity&0xFF00)>>8);                      //28
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.humidity&0x00FF);//29;//Humidity;          //29     
   /********************** Filter Status****************************/
   MODBUSFVFD.FillInput1[x++]=0;    //30
   MODBUSFVFD.FillInput1[x++]=GetAHUFilterStatus();//gu8Filter;    //31 
   /***************** Supply Air *******************************************/    
   uint32_t supplyair = sensor_card_data_rcv.sup_air_temp*10;//2500;//30*100;// 3000;     
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((supplyair & 0xFF000000)>>24); //32
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((supplyair & 0x00FF0000)>>16);        //33
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((supplyair & 0x0000FF00)>>8);         //34
   MODBUSFVFD.FillInput1[x++]= (uint8_t)(supplyair & 0x000000FF);            //35
   /***************** Amb Duct pressure  **************************************/    
   uint32_t ambductpress =(uint32_t)(u16Get_AvgPressure());     
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((ambductpress & 0xFF000000)>>24); //36
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((ambductpress & 0x00FF0000)>>16);     //37
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((ambductpress & 0x0000FF00)>>8);      //38
   MODBUSFVFD.FillInput1[x++]= (uint8_t)(ambductpress & 0x000000FF);           //39   
   /***************** voltage*******************************************/    
   uint32_t voltage =(GetRunningHours()/10);//(uint32_t)(u16Get_AvgVoltage());     
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((voltage & 0xFF000000)>>24); //40
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((voltage & 0x00FF0000)>>16);        //41
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((voltage & 0x0000FF00)>>8);         //42
   MODBUSFVFD.FillInput1[x++]= (uint8_t)(voltage & 0x000000FF);            //43 
   /***************** current*******************************************/    
   uint32_t current =(g1_GetRunningHours()/10);//(uint32_t)(u16Get_AvgCurrent());     
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((current & 0xFF000000)>>24); //44
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((current & 0x00FF0000)>>16);        //45
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((current & 0x0000FF00)>>8);         //46
   MODBUSFVFD.FillInput1[x++]= (uint8_t)(current & 0x000000FF);            //47
   /***************** running hours*******************************************/    
   uint32_t running_hours =(g2_GetRunningHours()/10);//(uint32_t)(NewRunningHours*100);      
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((running_hours & 0xFF000000)>>24); //48
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((running_hours & 0x00FF0000)>>16);   //49
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((running_hours & 0x0000FF00)>>8);     //50
   MODBUSFVFD.FillInput1[x++]= (uint8_t)(running_hours & 0x000000FF);         //51
   /********************** Fire signal****************************/
   MODBUSFVFD.FillInput1[x++]=0;    //52
   MODBUSFVFD.FillInput1[x++]=0;//53
   /*****************************VFD ON/OFF Source**************************/
   uint8_t u8stop_cause=0;
   if(ModbusRS485Params.u16Error>0)
   {
      u8stop_cause = ModbusRS485Params.u16Error+ERROR_OFFSET;
   }
   else
      u8stop_cause = fu8GetStopCondition();
   MODBUSFVFD.FillInput1[x++]=0;    //54
   MODBUSFVFD.FillInput1[x++]=u8stop_cause;//55
   /*****************************AQI**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.air_qual_ind&0xFF00)>>8);                      //56
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.air_qual_ind&0x00FF);       //57  
   /*****************************PM1.0**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.pm1p0&0xFF00)>>8);                      //58
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.pm1p0&0x00FF);       //59
   /*****************************PM10**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.pm10&0xFF00)>>8);                      //60
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.pm10&0x00FF);       //61
   /*****************************PM2.5**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.pm2p5&0xFF00)>>8);                      //62
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.pm2p5&0x00FF);       //63
   /*****************************tvoc**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.tvoc&0xFF00)>>8);                      //64
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.tvoc&0x00FF);       //65
    /*****************************carbon monoxide**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.co&0xFF00)>>8);                      //66
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.co&0x00FF);       //67
    /*****************************amonia**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.nh3&0xFF00)>>8);                      //68
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.nh3&0x00FF);       //69
   /*****************************nitrogen dioxide**************************/
    MODBUSFVFD.FillInput1[x++]=(uint8_t)((sensor_card_data_rcv.no2&0xFF00)>>8);                      //70
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(sensor_card_data_rcv.no2&0x00FF);       //71
     /*****************************AQI component**************************/
    MODBUSFVFD.FillInput1[x++]=0;                      //72
   MODBUSFVFD.FillInput1[x++]=sensor_card_data_rcv.air_qual_comp;       //73
    /*****************************UV Limit switch status**************************/
    MODBUSFVFD.FillInput1[x++]=0;                      //74
   MODBUSFVFD.FillInput1[x++]=fvGetUV_LS_Status();       //75
   /***************** UV running hours*******************************************/    
   uint32_t UVrunning_hours =(UV_GetRunningHours()/10);//(uint32_t)(NewRunningHours*100);      
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((UVrunning_hours & 0xFF000000)>>24); //76
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((UVrunning_hours & 0x00FF0000)>>16);   //77
   MODBUSFVFD.FillInput1[x++]= (uint8_t)((UVrunning_hours & 0x0000FF00)>>8);     //78
   MODBUSFVFD.FillInput1[x++]= (uint8_t)(UVrunning_hours & 0x000000FF);         //79
    /*****************************UV current Measured**************************/
   uint16_t uv_cur = (uint16_t)UV_get_current_data();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((uv_cur&0xFF00)>>8);                      //80
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(uv_cur&0x00FF);       //81
    /*****************************UV OffSource**************************/
   uint16_t uv_offsource = (uint16_t)fu8UVGetStopCondition();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((uv_offsource&0xFF00)>>8);                      //82
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(uv_offsource&0x00FF);       //83
    /*****************************UV Error**************************/
   uint16_t uv_error = (uint16_t)Get_UVOverCurrent_Error();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((uv_error&0xFF00)>>8);                      //84
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(uv_error&0x00FF);       //85
    /*****************************FAC OffSource**************************/
   uint16_t fac_offsource = 0;
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((fac_offsource&0xFF00)>>8);                      //86
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(fac_offsource&0x00FF);       //87
   
    /*****************************FAC OffSource**************************/
   uint16_t waterpress = u16Get_AvgWaterPressure();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((waterpress&0xFF00)>>8);                      //88
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(waterpress&0x00FF);       //89
   
   
    /*****************************GPIO**************************/
   uint16_t gpio1_s = GetAHUFilterStatus();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio1_s&0xFF00)>>8);                      //90
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio1_s&0x00FF);       //91
    
   uint16_t gpio2_s = GetpanelStatus();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio2_s&0xFF00)>>8);                      //92
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio2_s&0x00FF);       //93
   
   uint16_t gpio3_s = pump_runStatus();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio3_s&0xFF00)>>8);                      //94
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio3_s&0x00FF);       //95
   
   
   uint16_t gpio4_s = vfd_alarmstatus();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio4_s&0xFF00)>>8);                      //96
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio4_s&0x00FF);       //97
   
   uint16_t gpio5_s = gpio5staus();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio5_s&0xFF00)>>8);                      //98
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio5_s&0x00FF);       //99
   
   uint16_t gpio6_s = GetsmokeStatus();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio6_s&0xFF00)>>8);                      //100
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio6_s&0x00FF);       //101
   
   uint16_t gpio7_s = Get_gpio7();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio7_s&0xFF00)>>8);                      //102
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio7_s&0x00FF);       //103
   
   uint16_t gpio8_s = Get_gpio8();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio8_s&0xFF00)>>8);                      //104
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio8_s&0x00FF);       //105
   
   uint16_t gpio9_s = Get_gpio9();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio9_s&0xFF00)>>8);                      //106
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio9_s&0x00FF);       //107
   
   uint16_t gpio10_s = Get_gpio10();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio10_s&0xFF00)>>8);                      //108
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio10_s&0x00FF);       //109
   
   uint16_t gpio11_s = Get_gpio11();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio11_s&0xFF00)>>8);                      //110
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio11_s&0x00FF);       //111
   
   uint16_t gpio12_s = Get_gpio12();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio12_s&0xFF00)>>8);                      //112
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio12_s&0x00FF);       //113
   
   uint16_t gpio13_s = Get_gpio13();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio13_s&0xFF00)>>8);                      //114
   MODBUSFVFD.FillInput1[x++]=(uint8_t)(gpio13_s&0x00FF);       //115
   
   uint16_t gpio14_s = Get_gpio14();
   MODBUSFVFD.FillInput1[x++]=(uint8_t)((gpio14_s&0xFF00)>>8);                      //116
   MODBUSFVFD.FillInput1[x]=(uint8_t)(gpio14_s&0x00FF);       //117
   
   
   
}
int SetVFDHoldingParams(uint16_t Address,uint32_t ValueToWrite)
{
  fvRS485ParamsRead(&ModbusRS485Params);
  fvMenuItemsRead(&ModbusMenuItems);
  uint8_t FloatMulFact=100;
  uint8_t PID_mul_fact=10;
  uint8_t NumOfReg=0;
  if( Address == maMODESELEST)
  {
    if(/*(ValueToWrite>=0)&&*/(ValueToWrite<=3))
    {
      if( ((uint16_t)ValueToWrite)!= (ModbusMenuItems.u16AutoMan))
      {
        ModbusMenuItems.u16AutoMan = (uint16_t)ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
      }
      
      NumOfReg=1;
      vav_data_set_mode_for_all_vav((uint8_t)ValueToWrite);
    }else
    {
      NumOfReg=0;
    }
  }else{
  
    switch(Address) 
    {
    case maFREQUENCY:
      {
        if((ValueToWrite>=(ModbusMenuItems.u16MinFreq * 10))&&(ValueToWrite<=(ModbusMenuItems.u16MaxFreq * 10)))
        {
          SetBMSFailFlag(1);
          float fFreq = 0;                        
          fFreq = ((float)ValueToWrite)/100; 
          if(fu8FreqSet(fFreq))
          {
            NumOfReg =2;
          }
          else
            NumOfReg =0; 
        }else
        {
          NumOfReg=0;
        }
      }
     
      break;
    case maDIAGPACK:
      NumOfReg=1;
      break;
     
    case maSET_TEMP_VFD:
      if((ValueToWrite>=(23*FloatMulFact)) && (ValueToWrite<=(28*FloatMulFact)))
      { 
        ModbusMenuItems.u16SetTemp = (ValueToWrite)/10;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg=2;
      }else
      { 
        NumOfReg=0;
      }
      break;
    case maH20VALVE:
      if(/*(ValueToWrite>=0) &&*/ (ValueToWrite<=100))
      {
        SetBMSFailFlag(1);
        if(fu8WaterValveSet(ValueToWrite))
        {                       
          NumOfReg=1;
        }
        else
        {
          NumOfReg = 0;
        }
      }
      else
        NumOfReg =0;
      break;
    case maFRESHAIR:
      if(/*(ValueToWrite>=0) &&*/ (ValueToWrite<=100))
      {
        Set_FreshAiractuator_pos_per((uint8_t)ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    case maSCHEDULE_ON_R:
      
      if(ModbusMenuItems.u16ScheduleOnOff == SCHEDULESET_ON)
      {
        if( ((((ValueToWrite&0x0000FF00)>>8)<=23) /*&& (((ValueToWrite&0x0000FF00)>>8)>=0)*/) &&  (((ValueToWrite&0x000000FF)<=59)  /*&& ((ValueToWrite&0x000000FF)>=0)*/) )
        {
          ModbusMenuItems.u16ScheduleONTime = ValueToWrite;
          fvMenuItemsWrite(&ModbusMenuItems);
          fvMenustore();
          NumOfReg=1;
        }
        else
          NumOfReg =0;
      }else{
        NumOfReg=0;
      }
      break;
    case maSCHEDULE_OFF_R:
      if(ModbusMenuItems.u16ScheduleOnOff == SCHEDULESET_ON)
      {
        if( ((((ValueToWrite&0x0000FF00)>>8)<=23) && (((ValueToWrite&0x0000FF00)>>8)>=0)) &&  (((ValueToWrite&0x000000FF)<=59)  && ((ValueToWrite&0x000000FF)>=0)) )
        {
         
           ModbusMenuItems.u16ScheduleOFFTime = ValueToWrite;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();
          NumOfReg=1;
        }
        else
          NumOfReg=0;  
      }else
      {
        NumOfReg=0;
      }
      
      break;
    case maMIN_FREQUENCY:
      if((ValueToWrite>=(10*FloatMulFact)) && (ValueToWrite<=(50*FloatMulFact)))
      {
        ModbusMenuItems.u16MinFreq = ValueToWrite/10;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg =2;
      }
      else 
        NumOfReg =0 ;
      break;
    case maMAX_FREQUENCY:
      if((ValueToWrite>=(10*FloatMulFact)) && (ValueToWrite<=(50*FloatMulFact)))
      {
        ModbusMenuItems.u16MaxFreq = ValueToWrite/10;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg=2;
      }
      else
        NumOfReg =0 ;
      break;
    case maPID_CONST:
      if((ValueToWrite>=(0*PID_mul_fact)) && (ValueToWrite<=(5*PID_mul_fact)))
      {
        ModbusMenuItems.u16PIDConst = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg =2;
      }
      else
        NumOfReg=0;
      break;
    case maFLOW_SPAN:
      if((ValueToWrite>=0) && (ValueToWrite<=5))
      {
        if(ModbusMenuItems.u16FlowmeterType == MA_4_20)
        {
          if((ValueToWrite>=0) && (ValueToWrite<=1))
          {
            ModbusMenuItems.u16Span1 = ValueToWrite;
            fvMenuItemsWrite(&ModbusMenuItems);
            fvMenustore();
            NumOfReg =1;
          }
          else 
            NumOfReg=0;
        }
       else if(ModbusMenuItems.u16FlowmeterType == V_0_10)
        {
          if((ValueToWrite>=0) && (ValueToWrite<=4))
          {
            ModbusMenuItems.u16Span2 = ValueToWrite;
            fvMenuItemsWrite(&ModbusMenuItems);
            fvMenustore();
            NumOfReg =1;
          }
          else
            NumOfReg=0;
        }
        else
        {
          if((ValueToWrite>=0) && (ValueToWrite<=5))
          {
            ModbusMenuItems.u16Span3 = ValueToWrite;
            fvMenuItemsWrite(&ModbusMenuItems);
            fvMenustore();
            NumOfReg =1;
          }
          else
            NumOfReg=0;
        }
      }
      else
        NumOfReg=0;
      break;
    case maDATE_VFD:
      if(ValueToWrite!=0)
      {
        
      uint16_t year = (uint16_t)(ValueToWrite&0x0000FFFF);
      ModbusDate.Year = (uint8_t)(year-2000);
      ModbusDate.Month = (uint8_t)((ValueToWrite&0x00FF0000)>>16);
      ModbusDate.Date = (uint8_t)((ValueToWrite&0xFF000000)>>24);
      HAL_RTC_SetDate(&hrtc,&ModbusDate,RTC_FORMAT_BIN);
        NumOfReg =2;
      }else
        NumOfReg=0;
      break;
    case maTIME_VFD:
      if(( ((((ValueToWrite&0xFF000000)>>24)<=23) /*&& (((ValueToWrite&0xFF000000)>>24)>=0)*/) && ((((ValueToWrite&0x00FF0000)>>16)<=59) /*&& (((ValueToWrite&0x00FF0000)>>16)>=0)*/) && ((((ValueToWrite&0x0000FF00)>>8)<=59) && (((ValueToWrite&0x0000FF00)>>8)>=0)) ))
      {
        
      ModbusTime.Hours = (uint8_t)((ValueToWrite&0xFF000000)>>24);
      ModbusTime.Minutes = (uint8_t)((ValueToWrite&0x00FF0000)>>16);
      ModbusTime.Seconds = (uint8_t)((ValueToWrite&0x0000FF00)>>8);
      HAL_RTC_SetTime(&hrtc,&ModbusTime,RTC_FORMAT_BIN);
        NumOfReg=2;
      }
      else
        NumOfReg =0;
      break;
    case maSET_CO2:
      if((ValueToWrite>=100) && (ValueToWrite<=2000))
      {
        ModbusMenuItems.u16SetCO2 = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
      
    case maSET_DUCTPRESS:
      if(/*(ValueToWrite>=0) &&*/ (ValueToWrite<= (1250 * 10)))
      {
        ModbusMenuItems.u16DuctSetPressure = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg = 2;
      }
      else
        NumOfReg =0;
      break;
      
    case maSETWATER_DELTA_T:
      if(/*(ValueToWrite>=0) &&*/ (ValueToWrite<= (10 * 10)))
      {
        ModbusMenuItems.u16WaterDeltaT = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg = 2;
      }
      else
        NumOfReg =0;
      break;
      
    case maSETMAX_FLOWRATE:
      if(/*(ValueToWrite>=0) &&*/ (ValueToWrite<= (25 * 10)))
      {
        ModbusMenuItems.u16MaxFlowrate = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();
        NumOfReg = 2;
      }
      else
        NumOfReg =0;
      break;
    case maSET_TOTAL_VAV:
      if(/*(ValueToWrite>=0) && */(ValueToWrite<=32))
      {
        ModbusMenuItems.u16VavNumber = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        //calls_no_of_vavs
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break; 

    case maFlowmeter_Select:
      if((ValueToWrite == 0) || (ValueToWrite == 1) || (ValueToWrite == 2))
      {
        ModbusMenuItems.u16FlowmeterType = ValueToWrite;
        fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break; 
      
    case maVFD_Type:
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {
       ModbusMenuItems.u16VfdType = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
    case maDuct_Press_Span:
      if(/*(ValueToWrite >= 0) && */(ValueToWrite <= 5))
      {
       ModbusMenuItems.u16DuctPressureSpan = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
     
     case maPresConst:
      if(/*(ValueToWrite >= 0) && */(ValueToWrite <= 20))
      {
       ModbusMenuItems.u16PressureConstant = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
      case maInletThres:
      if(/*(ValueToWrite >= 0) && */(ValueToWrite <= 150))
      {
       ModbusMenuItems.u16Inlet_threshold = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
      case maUVAHUCont:
      if((ValueToWrite == 0) || (ValueToWrite ==1))
      {
       ModbusMenuItems.u16UV_AHU_Control = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
      case maUVCurThreshold:
      if(/*(ValueToWrite >= 0) && */(ValueToWrite <= 500))
      {
   //    ModbusMenuItems.u16uv_cur_threshold = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
      case maUVCTSel:
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {
     //  ModbusMenuItems.u16uv_ct_selection = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
       case maUVLmtSwchCont:
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {
     //  ModbusMenuItems.u16uv_limit_sw_control = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
      
      case maUVSchStatus:
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {
     //  ModbusMenuItems.u16uv_schedule_status = ValueToWrite;
       fvMenuItemsWrite(&ModbusMenuItems);
        fvMenustore();            
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;  
      case maUVSchONTime:
      
     /* if(ModbusMenuItems.u16uv_schedule_status == SCHEDULESET_ON)
      {
        if( ((((ValueToWrite&0x0000FF00)>>8)<=23) ) &&  (((ValueToWrite&0x000000FF)<=59)  ) )
        {
          ModbusMenuItems.u16uv_schedule_on_time = ValueToWrite;
          fvMenuItemsWrite(&ModbusMenuItems);
          fvMenustore();
         
          NumOfReg=1;
        }
        else
          NumOfReg =0;
      }else{
        NumOfReg=0;
      }
      break;
    case maUVSchOFFTime:
      if(ModbusMenuItems.u16uv_schedule_status == SCHEDULESET_ON)
      {
        if( ((((ValueToWrite&0x0000FF00)>>8)<=23) && (((ValueToWrite&0x0000FF00)>>8)>=0)) &&  (((ValueToWrite&0x000000FF)<=59)  && ((ValueToWrite&0x000000FF)>=0)) )
        {
         
           ModbusMenuItems.u16uv_schedule_off_time = ValueToWrite;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();

          NumOfReg=1;
        }
        else
          NumOfReg=0;  
      }else
      {
        NumOfReg=0;
      }
      
      break;
      case maUVStatus:
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {
       fvUVSetBmsStatus((uint8_t)ValueToWrite);
    volatile uint8_t  UVstatus = fu8UVGetMachineStartState((void*)(&ModbusMenuItems));
      
      if((UVstatus == 0) || (UVstatus == 1))
      {
          Set_UVOverCurrent_Clear();
          fvUVSetBmsStatus(UVstatus);
          fvSetUVStatus(UVstatus);
      }         
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break; */  
      case maFacSpeed:
      if((ValueToWrite == 0) || (ValueToWrite == 1)||(ValueToWrite == 2) || (ValueToWrite == 3))
      {        
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break;   
      case maFacSchStatus:
      if((ValueToWrite == 0) || (ValueToWrite == 1))
      {        
        NumOfReg=1;
      }
      else
        NumOfReg =0;
      break; 
      case maFacSchONTime:
      
      if(1)
      {
        if( ((((ValueToWrite&0x0000FF00)>>8)<=23) /*&& (((ValueToWrite&0x0000FF00)>>8)>=0)*/) &&  (((ValueToWrite&0x000000FF)<=59)  /*&& ((ValueToWrite&0x000000FF)>=0)*/) )
        {
         
          NumOfReg=1;
        }
        else
          NumOfReg =0;
      }else{
        NumOfReg=0;
      }
      break;
    case maFacSchOffTime:
      if(1)
      {
        if( ((((ValueToWrite&0x0000FF00)>>8)<=23) && (((ValueToWrite&0x0000FF00)>>8)>=0)) &&  (((ValueToWrite&0x000000FF)<=59)  && ((ValueToWrite&0x000000FF)>=0)) )
        {
          NumOfReg=1;
        }
        else
          NumOfReg=0;  
      }else
      {
        NumOfReg=0;
      }
      break;
       case maBTUSelection:
      if((ValueToWrite>=0) && (ValueToWrite<=1))
      {
         ModbusMenuItems.u16BtuType = ValueToWrite;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    case maMinWaterValvePos:
      if((ValueToWrite>=0) && (ValueToWrite<=4000))
      {
        ModbusMenuItems.u16MinValvePos = ValueToWrite/10;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    case maMinFlowRate:
      if((ValueToWrite>=0) && (ValueToWrite<=5000))
      {
        ModbusMenuItems.u16MinFlowrate = ValueToWrite/10;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    case maWaterPressSpan:
      if((ValueToWrite>=0) && (ValueToWrite<=5000))
      {
        ModbusMenuItems.u16WaterBar = ValueToWrite/10;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    case maFlowMeterSpan:
      if((ValueToWrite>=0) && (ValueToWrite<=5000))
      {
         ModbusMenuItems.u16MaxFlowSpan = ValueToWrite/10;
           fvMenuItemsWrite(&ModbusMenuItems);
           fvMenustore();
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    case ma1relay0:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
        fvModbusWriteVfd_pump(1,0,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
     case ma1relay1:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {

        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}        
        fvModbusWriteVfd_pump(1,1,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
       case ma1relay2:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
       fvModbusWriteVfd_pump(1,2,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
      
       case ma1relay3:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
       fvModbusWriteVfd_pump(1,3,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
      
        case ma2relay0:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
       // pump_prs.pressure1 = ValueToWrite;
        fvModbusWriteVfd_pump(2,0,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
     case ma2relay1:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {        
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
        fvModbusWriteVfd_pump(2,1,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
       case ma2relay2:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
       fvModbusWriteVfd_pump(2,2,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
     case ma2relay3:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
       fvModbusWriteVfd_pump(2,3,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
      
      
       case ma3relay0:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
        fvModbusWriteVfd_pump(3,0,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
     case ma3relay1:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      { 
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}       
        fvModbusWriteVfd_pump(3,1,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
       case ma3relay2:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
       fvModbusWriteVfd_pump(3,2,ValueToWrite);
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
      
       case ma3relay3:
      if((ValueToWrite>=0) && (ValueToWrite<=65535))
      {
        
        if(ValueToWrite>0){ValueToWrite=65280;}
        else {ValueToWrite=0;}
        fvModbusWriteVfd_pump(3,3,ValueToWrite);
       // pump_prs.pressure5 = ValueToWrite;
        NumOfReg=1;
      }
      else
        NumOfReg=0;
      break;
    default:
      NumOfReg=0;
      break;   
    } 
  }
  return NumOfReg;
}
void clear_data()
{
memset(&pump_prs,0,sizeof(pump_prs));
}
void ExceptionSend(uint8_t ExpType, uint8_t FuncCode) 
{
  stat_to_send[7]= FuncCode+0x80;
  uint16_t errorvalue = (uint16_t)((ExpType+BMSERROROFFSET)) ;
  stat_to_send[8] = (uint8_t)((errorvalue & 0xFF00)>>8);
  stat_to_send[9] =  (uint8_t)(errorvalue & 0x00FF);
  
}