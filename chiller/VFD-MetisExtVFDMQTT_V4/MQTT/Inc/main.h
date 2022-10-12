/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_it.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "UV.h"
#define PANEL_INPUT_SWITCH //uncomment for the panel with contactor input   
#define SYSTEMTICK_PERIOD_MS    1  
  
#define ALGO_ADDRESS 36
  
#define ALGO_SIGN 0X1234
  
__packed 
struct stVfdInputs
{
  uint16_t u16signature;
  uint8_t u8Switch : 1;
  uint8_t u8PanelSwitch : 1;
  uint8_t u8Schedule : 1;
  uint8_t u8BmsOnOff : 1;
  uint8_t u8exttrigger : 1;
  uint8_t u8automanual : 1;
  
};   

uint16_t fu16SendADCCommand(uint8_t* u8TxBuffer,uint8_t u8Length);
void TimeUpdate(void);
uint32_t Get_LocalTime(void);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern RTC_HandleTypeDef hrtc;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Water_In_Pin GPIO_PIN_6
#define Water_In_GPIO_Port GPIOA
#define UV_Current_Pin GPIO_PIN_7
#define UV_Current_GPIO_Port GPIOA
#define Return_Air_Pin GPIO_PIN_4
#define Return_Air_GPIO_Port GPIOC
#define Water_Out_Pin GPIO_PIN_5
#define Water_Out_GPIO_Port GPIOC
#define FREQ_CTRL_Pin GPIO_PIN_9
#define FREQ_CTRL_GPIO_Port GPIOE
#define SPI_CS_Pin GPIO_PIN_15
#define SPI_CS_GPIO_Port GPIOB
#define AHU_filter_Pin GPIO_PIN_6
#define AHU_filter_GPIO_Port GPIOC
#define Uart3_driver_pin_Pin GPIO_PIN_9
#define Uart3_driver_pin_GPIO_Port GPIOC
#define EXT_VFD_ON_Pin GPIO_PIN_1
#define EXT_VFD_ON_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
I2C_HandleTypeDef * fI2Cptrreturn(void);
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
