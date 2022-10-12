#include "Retrofit.h"
#include "stm32f1xx_hal.h"
#include "Keypad_Driver.h"
#include "modbus_master.h"

void fvWaterValveApply(uint16_t u16ValvePos)
{
      float fCount = 0;
      uint16_t temp = 0;
      temp = u16ValvePos;
      fCount = ((float)(500 * temp/4095));
      TIM1->CCR2 = (uint16_t)fCount;
}
void fvFreqApply(float fFreq)
{
  float data1=0;
  data1= fFreq;
  //fvModbusWriteFrq(data1); 
 // fvModbusWriteFrq_pump(2,data1);
  float fCount = 0;
      fCount = (float)(fFreq*5000/500);
      TIM1->CCR1 = (uint16_t)fCount;
      Write_VFDFreq(fFreq);
}
