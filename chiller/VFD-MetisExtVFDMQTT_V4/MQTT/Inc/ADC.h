/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/*********************** Macro for Temperature sensor*************************/
#define RESISTANCE_10K    10000
#define BETA_VALUE        3977
#define ROOM_TEMP_KELVIN  298.15
#define CONVERSION_K_TO_DEG 273.15  
#define NTC_RESISTANCE      3000
#define NUMSAMPLES          100
   

void Read_temp(uint16_t Count);
uint32_t* Get_ADC_DATA();
void FlowCalculation0_10V(uint16_t u16Count);
void FlowCalculation4_20MA(uint16_t u16Count);
void Pressure_Calculation(uint16_t Count);
void Water_DP_Calculation4_20MA(uint16_t u16Count);
void FlowCalculation5_10V(uint16_t u16Count);

void ntc_gpio(uint16_t u16Count);
void extra_gpio11(uint16_t u16Count);
void extra_gpio12(uint16_t u16Count);
void extra_gpio13(uint16_t u16Count);
void extra_gpio14(uint16_t u16Count);

float GetDuctPressure(void);
float Get_Water_In(void);
float Get_Water_Out(void);
float Get_Return_Air(void);
float Get_Flowrate(void);
float Get_Water_DP(void);
float Get_Sup_temp(void);

uint16_t Get_gpio7(void);
uint16_t Get_gpio8(void);
uint16_t Get_gpio9(void);
uint16_t Get_gpio10(void);

uint16_t Get_gpio11(void);
uint16_t Get_gpio12(void);
uint16_t Get_gpio13(void);
uint16_t Get_gpio14(void);

void UV_Current_Measure();


uint32_t UV_get_current_data();
#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/