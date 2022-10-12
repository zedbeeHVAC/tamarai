#include "TFTP_FlashConf.h"
#include "flash_if.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_flash.h"
#include "main.h"
#include "FRAM.h"
#include "Menu.h"
#define NUM_SET               14
struct stTFTP_Network_Params BoardParams;
struct stTFTP_FRAM_Params TFTP_Fram_Params;
struct stTFTP_ONTime_OFFTime tftp_ontime_offtime;
struct stTFTP_Date tftp_date;
static struct TstMenuItems tftpgstmenuitem;
static RTC_TimeTypeDef tftpTime;
static RTC_DateTypeDef tftpDate;
void fvTFTP_NetworkParamsRead(struct stTFTP_Network_Params *TFTPParams)
{
  memcpy(TFTPParams,&BoardParams,sizeof(struct stTFTP_Network_Params));
}
void fvTFTP_NetworkParamsWrite(struct stTFTP_Network_Params *TFTPParams)
{
  memcpy(&BoardParams,TFTPParams,sizeof(struct stTFTP_Network_Params));
}

void populateparams(void)
{     
  getpasscode(&BoardParams.passcode);
  if(BoardParams.passcode == VALIDPASSCODE) 
  {
    getipaddress(BoardParams.ipaddress);    
    getsubnetmask(BoardParams.subnetmask);   
    getgateway(BoardParams.gateway); 
    getserverip(BoardParams.serverip);
    //getUUID(&BoardParams.UUID);    
  }
  else 
  {
    BoardParams.passcode=VALIDPASSCODE;
    
    BoardParams.ipaddress[0]=192;
    BoardParams.ipaddress[1]=168;
    BoardParams.ipaddress[2]=0;
    BoardParams.ipaddress[3]=225; 
    
    BoardParams.subnetmask[0]=255;
    BoardParams.subnetmask[1]=255; 
    BoardParams.subnetmask[2]=255;
    BoardParams.subnetmask[3]=0;
    
    BoardParams.gateway[0]=192;
    BoardParams.gateway[1]=168;
    BoardParams.gateway[2]=0;
    BoardParams.gateway[3]=1;
    
    BoardParams.serverip[0]=0;//121;
    BoardParams.serverip[1]=0;//242;
    BoardParams.serverip[2]=0;//232;
    BoardParams.serverip[3]=0;//175;
    SetAddresses();
  }
}

void CmdDecoder(char *CmdString)  
{
   char stdconf[NUM_SET][25]={"setipadr","setsubnet","setgat","setservip","TFTPRST","BTU","RUNHOURS","AHUONTIME","AHUOFFTIME","DATE","TIME","UVONTIME","UVOFFTIME","UVRUNHOURS"};
   char Cmd_Str[150];//80
   int i;
   int option = -1;
   ExtractCmdStr(CmdString,Cmd_Str);
   /*Comparing extracted string with the std_conf strings */
   
   for(i=0;i<NUM_SET;i++)
   {  
      if(0==strcmp(Cmd_Str,stdconf[i]))
      {
         option=i;
         break;
      }
   }
   switch(option)
   {
   case 0:ExtractAddress(CmdString, BoardParams.ipaddress); 
   break;
   case 1:ExtractAddress(CmdString,BoardParams.subnetmask);          
   break;        
   case 2:ExtractAddress(CmdString,BoardParams.gateway);    
   break;
   case 3:ExtractAddress(CmdString,BoardParams.serverip);
   break;
   case 4:HAL_NVIC_SystemReset();
   break;
   case 5:ExtractId(CmdString,&TFTP_Fram_Params.BTU);
          TFTP_Write_BTU((TFTP_Fram_Params.BTU/1000));
   break;
   case 6:ExtractId(CmdString,&TFTP_Fram_Params.Running_Hours);
          TFTP_Write_RunHours(TFTP_Fram_Params.Running_Hours);
   break;
   case 7:ExtractOnTimeOffTime(CmdString,tftp_ontime_offtime.ontime);
   fvMenuItemsRead(&tftpgstmenuitem);
       tftpgstmenuitem.u16ScheduleONTime = (uint16_t)((tftp_ontime_offtime.ontime[0]<<8) + tftp_ontime_offtime.ontime[1]);
      fvMenuItemsWrite(&tftpgstmenuitem);
      fvMenustore(); 
      
         
   break;
   case 8:ExtractOnTimeOffTime(CmdString,tftp_ontime_offtime.offtime);
   fvMenuItemsRead(&tftpgstmenuitem);
       tftpgstmenuitem.u16ScheduleOFFTime = (uint16_t)((tftp_ontime_offtime.offtime[0]<<8) + tftp_ontime_offtime.offtime[1]);
      fvMenuItemsWrite(&tftpgstmenuitem);
      fvMenustore();    
       
   break;
    case 9:ExtractDate(CmdString,tftp_date.date);
      tftpDate.Year = (uint8_t)(tftp_date.date[2]-2000);
     tftpDate.Month = (uint8_t)(tftp_date.date[1]);
     tftpDate.Date = (uint8_t)(tftp_date.date[0]);  
      HAL_RTC_SetDate(&hrtc,&tftpDate,RTC_FORMAT_BIN);
   break;
   case 10:ExtractTime(CmdString,tftp_ontime_offtime.time);
     tftpTime.Hours = (uint8_t)(tftp_ontime_offtime.time[0]);
     tftpTime.Minutes = (uint8_t)(tftp_ontime_offtime.time[1]);
     tftpTime.Seconds = (uint8_t)(tftp_ontime_offtime.time[2]);   
     HAL_RTC_SetTime(&hrtc,&tftpTime,RTC_FORMAT_BIN);
   break;
   case 11:ExtractOnTimeOffTime(CmdString,tftp_ontime_offtime.uvontime);
    fvMenuItemsRead(&tftpgstmenuitem);
     //  tftpgstmenuitem.u16uv_schedule_on_time = (uint16_t)((tftp_ontime_offtime.uvontime[0]<<8) + tftp_ontime_offtime.uvontime[1]);
      fvMenuItemsWrite(&tftpgstmenuitem);
      fvMenustore(); 
      
     break;
   case 12:ExtractOnTimeOffTime(CmdString,tftp_ontime_offtime.uvofftime);
    fvMenuItemsRead(&tftpgstmenuitem);
     //  tftpgstmenuitem.u16uv_schedule_off_time = (uint16_t)((tftp_ontime_offtime.uvofftime[0]<<8) + tftp_ontime_offtime.uvofftime[1]);
       fvMenuItemsWrite(&tftpgstmenuitem);
      fvMenustore();    
      
     break;
      case 13:ExtractId(CmdString,&TFTP_Fram_Params.UVRunning_Hours);
          UV_TFTP_Write_RunHours(TFTP_Fram_Params.UVRunning_Hours);
   break;
   //case 5:ExtractId(CmdString,&BoardParams.UUID);
   //break;
   default:
      break;
     
   }
}
void SetAddresses(void)
{
   uint32_t Buffer[(uint16_t)((sizeof(BoardParams))/4)]={0};
   
   int i=0,j=0;
   
   BoardParams.passcode=VALIDPASSCODE;
   Buffer[0] = BoardParams.passcode;
   i=1;
   for(j=0;j<4;j++)
   {
      Buffer[i++]=BoardParams.ipaddress[j];
   }
   
   for(j=0;j<4;j++)
   {
      Buffer[i++]=BoardParams.subnetmask[j];
   }
   
   for(j=0;j<4;j++)
   {
      Buffer[i++]=BoardParams.gateway[j];
   }
   for(j=0;j<4;j++)
   {
   Buffer[i++] = BoardParams.serverip[j];
   }
   //Buffer[i++]= BoardParams.UUID;
  FLASH_If_Init();
  FLASH_If_Erase(TFTPPARMS_FLASH_START_ADDRESS, 1);
  uint32_t Address=TFTPPARMS_FLASH_START_ADDRESS;
  FLASH_If_Write(&Address,Buffer,(uint16_t)((sizeof(BoardParams))/4));
  FLASH_If_DeInit(); 
}
void ExtractCmdStr(char *string, char *sub_string)
{
  int i=0;
  char *temp;
  temp=string;

  while((*temp)!=('\0')&&(*temp)!=(' '))
  {
     sub_string[i]=*temp;
     temp++;
     i++;
  }
  sub_string[i]='\0';
}
void ExtractAddress(char *string, uint32_t *address)
{
  char *temp;
  temp=string;

  while((*temp)!=' ')temp++;
  temp++;

  FormAddress(temp,address);
}
void FormAddress(char *temp, uint32_t *address)
{
  char number[5];
  int i=0;
  int add_index=0;

  while(((*temp)!='\0'))
  {
    while(( (*temp)!=':' )&&( (*temp)!='.' )&&( (*temp)!='\0' ))
    {
      number[i]=*temp;
      //printf("%c\n",number[i]);
      temp++;
      i++;
    }
    number[i]='\0';
    //printf("\nnumber---%s\n",number);
    i=0;
    address[add_index++]=(uint32_t)atoi(number);  //converting address from ascii to interger and moving to add_index++ and so incrementing
    if( ((*temp)=='.')||((*temp)==':') ) 
    temp++;
  }

}
void ExtractOnTimeOffTime(char *string, uint8_t *address)
{
  char *temp;
  temp=string;

  while((*temp)!=' ')temp++;
  temp++;

  FormOnTimeOffTime(temp,address);
}
void FormOnTimeOffTime(char *temp, uint8_t *address)
{
  char number[3];
  int i=0;
  int add_index=0;

  while(((*temp)!='\0'))
  {
    while(( (*temp)!=':' )&&( (*temp)!='.' )&&( (*temp)!='\0' ))
    {
      number[i]=*temp;
      //printf("%c\n",number[i]);
      temp++;
      i++;
    }
    number[i]='\0';
    //printf("\nnumber---%s\n",number);
    i=0;
    address[add_index++]=(uint8_t)atoi(number); 
    if( ((*temp)=='.')||((*temp)==':') ) 
    temp++;
  }

}
void ExtractTime(char *string, uint8_t *address)
{
  char *temp;
  temp=string;

  while((*temp)!=' ')temp++;
  temp++;

  FormTime(temp,address);
}
void FormTime(char *temp, uint8_t *address)
{
  char number[4];
  int i=0;
  int add_index=0;

  while(((*temp)!='\0'))
  {
    while(( (*temp)!=':' )&&( (*temp)!='.' )&&( (*temp)!='\0' ))
    {
      number[i]=*temp;
      //printf("%c\n",number[i]);
      temp++;
      i++;
    }
    number[i]='\0';
    //printf("\nnumber---%s\n",number);
    i=0;
    address[add_index++]=(uint8_t)atoi(number); 
    if( ((*temp)=='.')||((*temp)==':') ) 
    temp++;
  }

}
void ExtractDate(char *string, uint16_t *address)
{
  char *temp;
  temp=string;

  while((*temp)!=' ')temp++;
  temp++;

  FormDate(temp,address);
}
void FormDate(char *temp, uint16_t *address)
{
  char number[4];
  int i=0;
  int add_index=0;

  while(((*temp)!='\0'))
  {
    while(( (*temp)!=':' )&&( (*temp)!='.' )&&( (*temp)!='\0' ))
    {
      number[i]=*temp;
      //printf("%c\n",number[i]);
      temp++;
      i++;
    }
    number[i]='\0';
    //printf("\nnumber---%s\n",number);
    i=0;
    address[add_index++]=(uint16_t)atoi(number); 
    if( ((*temp)=='.')||((*temp)==':') ) 
    temp++;
  }

}
void getipaddress(uint32_t *ipaddressbagempty)
{
 FLASH_If_Init(); 
 uint32_t Address=FLASH_ADDR_IPADDRESS;
 FLASH_If_Read(&Address,ipaddressbagempty,4);
 FLASH_If_DeInit();
}

/******************************************************************************/
void getsubnetmask(uint32_t *subnetbagempty)
{
 FLASH_If_Init(); 
 uint32_t Address=FLASH_ADDR_SUBNETMASK;
 FLASH_If_Read(&Address,subnetbagempty,4);
 FLASH_If_DeInit(); 

}

/******************************************************************************/
void getgateway(uint32_t *gatewaybagempty)
{
 FLASH_If_Init(); 
 uint32_t Address=FLASH_ADDR_GATEWAY;
 FLASH_If_Read(&Address,gatewaybagempty,4);
 FLASH_If_DeInit();  
 
}
/******************************************************************************/
void getpasscode(uint32_t *passcodebagempty)
{
 FLASH_If_Init(); 
 uint32_t Address=FLASH_ADDR_PASSCODE;
 FLASH_If_Read(&Address,passcodebagempty,1);
 FLASH_If_DeInit();
 
}

/******************************************************************************/
void getserverip(uint32_t *serveripbagempty)
{
 FLASH_If_Init(); 
 uint32_t Address=FLASH_ADDR_SERVERIP;
 FLASH_If_Read(&Address,serveripbagempty,4);
 FLASH_If_DeInit();
}
void getUUID(uint32_t *UUIDbagempty)
{
   FLASH_If_Init(); 
   uint32_t Address=FLASH_ADDR_UUID;
   FLASH_If_Read(&Address,UUIDbagempty,4);
   FLASH_If_DeInit();
}
uint8_t ExtractId(char *string, uint32_t *id)
{
  char *temp1;
  temp1=string;
  uint8_t size = 0;
  
  while((*temp1)!=' ')temp1++;
  temp1++;
  size = (uint8_t)FormId(temp1,id);
  return size;
}

/******************************************************************************************************************************************/

int FormId(char *temp1, uint32_t *id)
{
  char number[5];
  int i=0;
  int add_index=0;
  
  while((*temp1)!='\0')
  {
    while(((*temp1)!=' ') && ((*temp1)!='\0'))
    {
      number[i]=*temp1;
      temp1++;
      i++;
    }
    number[i]='\0';
    i=0;
    id[add_index++]=(uint32_t)atoi(number);
    if((*temp1)==' ')
      temp1++;
  }
  return add_index;
}