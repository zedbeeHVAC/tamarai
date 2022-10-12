/**
  ******************************************************************************
  * @file    flash_if.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    07/16/2010 
  * @brief   This file provides high level routines to manage internal Flash 
  *          programming (erase and write). 
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_flash.h"
#include "TFTP_FlashConf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{ 
  HAL_FLASH_Unlock(); 
}

void FLASH_If_DeInit(void)
{ 
  HAL_FLASH_Lock(); 
}
/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0 if success, -1 if error
  */
int8_t FLASH_If_Erase(uint32_t PageID, uint8_t NoofPages)
{
  uint32_t u32error = 0;
  
  FLASH_EraseInitTypeDef FLASH_EraseInit;
  
  FLASH_EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  FLASH_EraseInit.Banks = 0;
  FLASH_EraseInit.PageAddress = PageID;
  FLASH_EraseInit.NbPages = NoofPages;
  
  if(HAL_FLASHEx_Erase(&FLASH_EraseInit,&u32error) != HAL_OK)
  {
     return -1;
  }
  
  return 0;
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned)
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval None
  */
void FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t *Data ,uint16_t DataLength)
{
  uint32_t i = 0;
  
  for (i = 0; i < DataLength; i++)
  {
    if (*FlashAddress <= (TFTPPARMS_FLASH_END_ADDRESS-4))
    {
      uint32_t data_to_write=*(uint32_t*)(Data + i);
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,*FlashAddress,data_to_write) == HAL_OK)
      {
        *FlashAddress += 4;
      }
      else
      {
        //printf("Flash Error");
        return;
      }
    }
    else
    {
      //printf("End of page");
      return;
    }
  }
}

void FLASH_If_Read(__IO uint32_t* FlashAddress, uint32_t* Data, uint16_t DataLength)
{
 uint32_t i = 0;
  
 for(i=0;i < DataLength; i++)
 {
 if (*FlashAddress <= (TFTPPARMS_FLASH_END_ADDRESS-4))
    {
        Data[i]=*(__IO uint32_t*)(*FlashAddress);
        *FlashAddress += 4;
     
    }
    else
      return;
 
}
}





/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
