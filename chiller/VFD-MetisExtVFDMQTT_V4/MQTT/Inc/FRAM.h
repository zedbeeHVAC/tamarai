#ifndef __FRAM_H
#define __FRAM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define DEVICE_ADDRESS 0xAC

  
#define BTU_MEMORY_ADDRESS 20

  
#define UV_BTU_MEMORY_ADDRESS 28

  
#define BTU2_MEMORY_ADDRESS 40

  
#define UV2_BTU_MEMORY_ADDRESS 48


#define FILL_PRS 90  
#define SCALE_FACT 1000
#define GPIO_1_ADDRESS 100
#define GPIO_2_ADDRESS 104
#define GPIO_3_ADDRESS 108

  
  typedef struct stgpiostore
{
  uint32_t u32gpio_run;
}GPIOstore;
typedef struct stframstore
{
   uint32_t u32btu;
   uint32_t u32running;
 
   
}Tstframstore;

uint16_t FramRead_Fill_Prs();
void FramWrite_Fill_Prs(uint16_t data_write);

void BTU_RunningHourInit(void);
void FramReadBuff(uint16_t ReadAddr);
void FramReadBuff2(uint16_t ReadAddr);
void run2();
void btc_crc_check();
void uv_crc_check();
void uv_run();
void BTU_meter(void);
void BTU2_meter(void);
uint32_t GetBTU(void);
uint32_t GetRunningHours(void);
uint16_t fram_btu_crc(void );
void Running_HoursCalc(void);
void TFTP_Write_BTU(uint32_t btu);
void TFTP_Write_RunHours(uint32_t RunHours);
void UV_FramReadBuff(uint16_t ReadAddr);
void UV2_FramReadBuff(uint16_t ReadAddr);
uint32_t UV_GetRunningHours(void);
void UV_Running_HoursCalc(void);
void UV_TFTP_Write_RunHours(uint32_t RunHours);
void Algo_FramReadBuff(uint16_t ReadAddr,struct stVfdInputs* algoptr);
void Algo_FramWriteBuff(uint16_t WritaAddr,struct stVfdInputs* algoptr);

void FramGPIO1Buff(uint16_t ReadAddr);
void FramGPIO2Buff(uint16_t ReadAddr);
void FramGPIO3Buff(uint16_t ReadAddr);

uint32_t g1_GetRunningHours(void);
uint32_t g2_GetRunningHours(void);
uint32_t g3_GetRunningHours(void);

void gpio_Running_Hours(void);
#ifdef __cplusplus
}
#endif

#endif /*__FRAM_H*/