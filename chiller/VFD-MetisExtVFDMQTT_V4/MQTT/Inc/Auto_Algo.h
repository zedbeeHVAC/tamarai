#ifndef __AUTOALGO_H
#define __AUTOALGO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#define MAXVAV                32
   
uint8_t fu8WaterValveSet(uint8_t u8WaterPercent);
uint8_t fu8GetWaterValvePercent(void);
void freq_dec(void);
void freq_inc(void);
void fvFreqSet(void);
void fvDuctpress_logic(void);
void freq_check(void);
void fvWaterValveCtrl(void);
void fvVavCalc(void);
uint8_t fu8FreqSet(float fFreq);
void Mode_Switching(void);
void SetBMSFailFlag(uint8_t flag);
uint8_t GetBMSFailFlag(void);
void pressure_average();
void auto_pressure_freq();
float Minimum_Pressure(void);
uint16_t u16Minimum_Pressure(void);
#ifdef __cplusplus
}
#endif

#endif /* __AUTOALGO_H */