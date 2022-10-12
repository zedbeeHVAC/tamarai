#include "ADC.h"
#include "Timer.h"
#include "Menu.h"
#include "Math.h"
#include "ModbusProcess.h"
float Temp_11 = 0,Temp_22 = 0,Temp_33 = 0,Temp_44 = 0,In_total = 0,Out_total = 0,Ret_total=0,sup_total = 0;
float Temp_in=0,Temp_out=0,Temp_ret=0,sup_tem=0,Flow_sec=0,Flow_ms=0,Flow_lps=0;
float water_pres_vol=0,water_pres=0,water_press_curr=0;
float water_pres1=0,water_pres2=0,water_pres3=0,water_pres4=0,water_pres5=0;
float water_pres_vol1=0,water_pres_vol2=0,water_pres_vol3=0,water_pres_vol4=0,water_pres_vol5=0;
float water_press_curr1=0,water_press_curr2=0,water_press_curr3=0,water_press_curr4=0,water_press_curr5=0;
double Res_in = 0;
uint32_t gu32WaterDpAcc = 0;
uint32_t gu32FlowAcc = 0;
int16_t UV_Cur_data = 0;
double UV_Current = 0;
int16_t UV_Cur_Array[1024] = {0};
uint32_t UV_Cur_Avg = 0;
double UV_mVolt = 0;
double UV_Cur_Val = 0;
uint32_t ADC_DATA[6];
uint16_t UV_mesure_count = 0;
struct TstMenuItems ADCMenuParam;
struct PRESSURE pump_pressure;
uint8_t test_code = 0;
/*void UV_Current_Measure()
{
  
  fvMenuItemsRead(&ADCMenuParam);
  uint32_t *UV_Cur;
  UV_Cur = Get_ADC_DATA();
  UV_Cur_data = UV_Cur[3];
  UV_Cur_Array[UV_mesure_count++] = UV_Cur_data;
  
  
  UV_Cur_data = ( 1530 - UV_Cur_data );
  UV_Cur_Avg += ( UV_Cur_data * UV_Cur_data );
  if( UV_mesure_count > 999 )
  {
    UV_Cur_Avg = UV_Cur_Avg / UV_mesure_count ;
    UV_Cur_Val = sqrt(UV_Cur_Avg);
    UV_Cur_Avg = 0;
    UV_mesure_count = 0;
    UV_mVolt = (UV_Cur_Val  / 4095) * 3.3;
    if( ADCMenuParam.u16uv_ct_selection == UV_2A )
    {
      UV_Current = (2.6340841781261 * UV_mVolt) -0.0558363891829577;
     // UV_Current = ((( 6.962406 *  pow(UV_mVolt , 3)) + ( -9.94872 *  pow(UV_mVolt , 2) ) +( 7.252373 * UV_mVolt  )) - 0.91925 );
    }
    else if ( ADCMenuParam.u16uv_ct_selection == UV_5A)
    {
      if(UV_mVolt < 0.02)
      {
          UV_mVolt = 0;
      }
      UV_Current = (9.09145364978103 * UV_mVolt) - 0.0636657795044837;//(((344269.162749911 * pow(UV_mVolt , 5 )) + (-503273.725122283 * pow(UV_mVolt , 4) ) + ( 293428.511366562 *  pow(UV_mVolt , 3)) + ( -85323.7241786381 *  pow(UV_mVolt , 2) ) +( 12392.2658162173 * UV_mVolt  ))  -719.452199805599 );
    }
    if(UV_Current < 0)
    {
        UV_Current = 0;
    }
  
  }
  

}*/

uint32_t
UV_get_current_data()
{
  return UV_Current * 100;
}

uint32_t* Get_ADC_DATA()
{
  return ADC_DATA;
}
float pt100_res=0;
void Read_temp(uint16_t u16Count)
{
  uint32_t *data;
  data = Get_ADC_DATA();
  Temp_11+=data[0];
  Temp_22+=data[1];
  Temp_33+=data[2];
  Temp_44+=data[3];
  if(u16Count >= NUMSAMPLES)
  {
    In_total=Temp_11/u16Count;
    Out_total=Temp_22/u16Count;
    Ret_total=Temp_33/u16Count;
    sup_total=Temp_44/u16Count;

    Res_in = (In_total * 3.3/4095);
    Res_in = (1000*Res_in)/(5.001-Res_in);
    Res_in = (1000*Res_in)/(1000-Res_in);
    if(Res_in > 0)
    {
      pt100_res=Res_in;
     // Res_in = ((log(Res_in/3000))/3977)+(1/298.15);
    //  Temp_in = (1/Res_in)-273.15;

      Temp_in=( -1 * ((sqrt((-0.00232*Res_in)+17.59246)-3.908)/0.00116));// pt1000


     // Temp_in=(Res_in/100-1.0)/0.00385;  // pt100

    }
    else
    {
      Temp_in = 0;
    }
    if( Temp_in > 100 || Temp_in <= 0)
    {
      Temp_in = 0;
    }
    Res_in = (Out_total * 3.3/4095);
    Res_in = (1000*Res_in)/(5.001-Res_in);
    Res_in = (1000*Res_in)/(1000-Res_in);
    if(Res_in > 0)
    {
     //Res_in = ((log(Res_in/3000))/3977)+(1/298.15);
      //Temp_out = (1/Res_in)-273.15;
      Temp_out=( -1 * ((sqrt((-0.00232*Res_in)+17.59246)-3.908)/0.00116));
    }
    else
    {
      Temp_out = 0;
    }
    if( Temp_out > 100 || Temp_out <= 0)
    {
      Temp_out = 0;
    }
    Res_in = (Ret_total * 3.3/4095);
    Res_in = (1000*Res_in)/(5.001-Res_in);
    Res_in = (1000*Res_in)/(1000-Res_in);
    if(Res_in > 0)
    {
      
      //Res_in = ((log(Res_in/3000))/3977)+(1/298.15);
     // Temp_ret = (1/Res_in)-273.15;
      Temp_ret=( -1 * ((sqrt((-0.00232*Res_in)+17.59246)-3.908)/0.00116));
    }
    else
    {
    Temp_ret=0;
    }
     if( Temp_ret > 100 || Temp_ret <= 0)
    {
      Temp_ret = 0;
    }


     Res_in = (sup_total * 3.3/4095);
    Res_in = (1000*Res_in)/(5.001-Res_in);
    Res_in = (1000*Res_in)/(1000-Res_in);
    if(Res_in > 0)
    {
     // Res_in = ((log(Res_in/3000))/3977)+(1/298.15);
    //  Temp_in = (1/Res_in)-273.15;

      sup_tem=( -1 * ((sqrt((-0.00232*Res_in)+17.59246)-3.908)/0.00116));
//      Temp_in = sqrt(6);
   //   pt_res=Res_in;
    }
    else
    {
      sup_tem = 0;
    }
    if( sup_tem > 100 || sup_tem <= 0)
    {
      sup_tem = 0;
    }

    Temp_11=0;
    Temp_22=0;
    Temp_33=0;
    Temp_44=0;
  }
 }

void FlowCalculation0_10V(uint16_t u16Count)
{
  fvMenuItemsRead(&ADCMenuParam);
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x40;
  u8Command[2] = 0x00;
  float fspan = 0;
  gu32FlowAcc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  
  if(u16Count >= NUMSAMPLES)
  {
    Flow_sec=(float)((gu32FlowAcc)/u16Count);
    Flow_sec = Flow_sec * 12.5 /4095;
    fspan=ADCMenuParam.u16MaxFlowSpan/10;
   /* switch(ADCMenuParam.u16Span2)
    {
    case PIPE_25MM:
      fspan = 1.15;
      break;
    case PIPE_32MM:
      fspan = 1.8;
      break;
    case PIPE_40MM:
      fspan = 2.5;
      break;
    case PIPE_50MM:
      fspan = 4.8;
      break;
    default:
      fspan = 6.3;
      break;
    }*/
    if(Flow_sec >= 2)
    {
      Flow_ms= (float)((((((Flow_sec)-2))/8)*fspan));
      Flow_lps = Flow_ms;
    }
    else
    {
      Flow_ms = 0;
      Flow_lps = Flow_ms;
    }
    gu32FlowAcc=0;
    
  }
}
void FlowCalculation5_10V(uint16_t u16Count)
{
  fvMenuItemsRead(&ADCMenuParam);
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x40;
  u8Command[2] = 0x00;
  float fspan = 0;
  gu32FlowAcc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  
  if(u16Count >= NUMSAMPLES)
  {
    Flow_sec=(float)((gu32FlowAcc)/u16Count);
    Flow_sec = Flow_sec * 12.5 /4095;
    
    switch(ADCMenuParam.u16Span3)
    {
    case FM_15R:
      fspan = 0.42;
      break;
    case FM_20R:
      fspan = 0.78;
      break;
    case FM_25R:
      fspan = 1.38;
      break;
    case FM_32R:
      fspan = 2.16;
      break;
    case FM_40R:
       fspan = 3;
       break;
    case FM_50R:
       fspan = 5.76;
       break;
     case FM_65F:
      fspan = 9.6;
      break;
    case FM_80F:
      fspan = 13.6;
      break;
    case FM_100F:
       fspan = 24.0;
       break;
    case FM_125F:
       fspan = 37.5;
       break;
    default:
      fspan = 54.0;// FM_150F
      break;
    }
    if(Flow_sec >= 0.5)
    {
      Flow_ms= (float)((((((Flow_sec)-0.5))/9.5)*fspan));
      Flow_lps = Flow_ms;
    }
    else
    {
      Flow_ms = 0;
      Flow_lps = Flow_ms;
    }
    gu32FlowAcc=0;
    
  }
}
void FlowCalculation4_20MA(uint16_t u16Count)
{
  fvMenuItemsRead(&ADCMenuParam);
  float fspan = 0;
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x00;
  u8Command[2] = 0x00;
  gu32FlowAcc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    Flow_sec=(float)((gu32FlowAcc)/u16Count);
    Flow_sec = Flow_sec * 5/4095;
    switch(ADCMenuParam.u16Span1)
    {
    case SPAN_90:
      fspan = 5.625;
      break;
    default:
      fspan = 3.125;
      break;
    }
    if(Flow_sec >= 0.4)
    {
      Flow_ms= (float)((((((Flow_sec*10)-4))*fspan)));
    }
    else
    {
      Flow_ms = 0;
    }
    Flow_ms= (Flow_ms*0.278);                            // meter cube per hour to litere per second
    Flow_lps = Flow_ms;
    gu32FlowAcc=0;
  }
}

uint32_t readvalue = 0;
float dp_value = 0;
float Duct_pres_vol = 0;
float dp_measure_value = 0; 
void Pressure_Calculation(uint16_t Count)
{
  fvMenuItemsRead(&ADCMenuParam);
  uint8_t u8Command[3];
  float current=0;
  uint16_t fspan = 0;
  u8Command[0] = 0x06;
  u8Command[1] = 0xC0;//0x80;
  u8Command[2] = 0x00;
  readvalue += (fu16SendADCCommand(u8Command,3) & 0x0fff);
  
  /*switch(ADCMenuParam.u16DuctPressureSpan)
  {
  case SPAN_2500:
    fspan = 2500;
    break;
  case SPAN_2000:
    fspan = 2000;
    break;
  case SPAN_1500:
    fspan = 1500;
    break;
  case SPAN_1250:
    fspan = 1250;
    break;
  case SPAN_500:
    fspan = 500;
    break;
  default:
    fspan = 250;
    break;
  }*/
  
  if(Count >= NUMSAMPLES)
  {
     fspan= ADCMenuParam.u16FillBar/10;
    dp_value = (float)readvalue / NUMSAMPLES;
    Duct_pres_vol = (dp_value*5*1.006)/4095;
    current = (Duct_pres_vol /100)*1000;
    if(current >= 4)
    {
      dp_measure_value = (((current - 4)*fspan)/16);
    }
    else
    {
        dp_measure_value = 0;
    }
    readvalue = 0; 
  }
}

void Water_DP_Calculation4_20MA(uint16_t u16Count)
{
  fvMenuItemsRead(&ADCMenuParam);
  
  float fspan =0;
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x00;
  u8Command[2] = 0x00;
 gu32WaterDpAcc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    water_pres_vol=(float)((gu32WaterDpAcc)/u16Count);
    water_pres_vol = water_pres_vol * 5/4095;
    water_press_curr = (water_pres_vol/100)*1000;
   fspan= ADCMenuParam.u16WaterBar/10;
 
    if(water_press_curr > 4)
    {
      water_pres1= (((water_press_curr - 4)*fspan)/16);
    }
   else
    {
      water_pres1 = 0;
    }
 
    gu32WaterDpAcc=0;
  }
}

uint32_t Temp_7=0,Temp_8=0,Temp_9=0,Temp_10=0;
uint16_t gp7=0,gp8=0,gp9=0,gp10=0;
uint16_t gpiostatus7=0,gpiostatus8=0,gpiostatus9=0,gpiostatus10;
void ntc_gpio(uint16_t u16Count)
{
uint32_t *data;
  data = Get_ADC_DATA();
  Temp_7+=data[0];
  Temp_8+=data[1];
  Temp_9+=data[2];
  Temp_10+=data[3];
  if(u16Count >= NUMSAMPLES)
  {
    gp7=Temp_7/u16Count;
    gp8=Temp_8/u16Count;
    gp9=Temp_9/u16Count;
    gp10=Temp_10/u16Count;
    
     if(gp7>1500){ gpiostatus7=1;}
  else {gpiostatus7=0;}
  
     if(gp8>1500){ gpiostatus8=1;}
  else {gpiostatus8=0;}
  
  
     if(gp9>1500){ gpiostatus9=1;}
  else {gpiostatus9=0;}
  
  
     if(gp10>1500){ gpiostatus10=1;}
  else {gpiostatus10=0;}
  
Temp_7=0;Temp_8=0;Temp_9=0;Temp_10=0;
  }
}
/*uint32_t t1Acc=0,t2Acc=0,t3Acc=0,t4Acc=0;
void Temp1_pt100_4_20ma(uint16_t u16Count)
{
fvMenuItemsRead(&ADCMenuParam);
  float fspan =0;
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x40;
  u8Command[2] = 0x00;
  t1Acc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    water_pres_vol=(float)((t1Acc)/u16Count);
    water_pres_vol = water_pres_vol * 5/4095;
    water_press_curr = (water_pres_vol/100)*1000;
   fspan= 50;//ADCMenuParam.u16WaterBar/10;
 
    if(water_press_curr > 4)
    {
      Temp_in= (((water_press_curr - 4)*fspan)/16);
    }
   else
    {
      Temp_in = 0;
    }
  
    t1Acc=0;
  }
}
void Temp2_pt100_4_20ma(uint16_t u16Count)
{
fvMenuItemsRead(&ADCMenuParam);
  float fspan =0;
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0xC0;
  u8Command[2] = 0x00;
  t2Acc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    water_pres_vol=(float)((t2Acc)/u16Count);
    water_pres_vol = water_pres_vol * 5/4095;
    water_press_curr = (water_pres_vol/100)*1000;
   fspan= 50;//ADCMenuParam.u16WaterBar/10;
 
    if(water_press_curr > 4)
    {
      Temp_out= (((water_press_curr - 4)*fspan)/16);
    }
   else
    {
      Temp_out = 0;
    }
  
    t2Acc=0;
  }
}

void Temp3_pt100_4_20ma(uint16_t u16Count)
{
fvMenuItemsRead(&ADCMenuParam);
  float fspan =0;
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x00;
  u8Command[2] = 0x00;
  t3Acc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    water_pres_vol=(float)((t3Acc)/u16Count);
    water_pres_vol = water_pres_vol * 5/4095;
    water_press_curr = (water_pres_vol/100)*1000;
   fspan= 50;//ADCMenuParam.u16WaterBar/10;
 
    if(water_press_curr > 4)
    {
      Temp_ret= (((water_press_curr - 4)*fspan)/16);
    }
   else
    {
      Temp_ret = 0;
    }
  
    t3Acc=0;
  }
}


void Temp4_pt100_4_20ma(uint16_t u16Count)
{
fvMenuItemsRead(&ADCMenuParam);
  float fspan =0;
  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x80;
  u8Command[2] = 0x00;
  t4Acc += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    water_pres_vol=(float)((t4Acc)/u16Count);
    water_pres_vol = water_pres_vol * 5/4095;
    water_press_curr = (water_pres_vol/100)*1000;
   fspan= 50;//ADCMenuParam.u16WaterBar/10;
 
    if(water_press_curr > 4)
    {
      sup_tem= (((water_press_curr - 4)*fspan)/16);
    }
   else
    {
      sup_tem = 0;
    }
  
    t4Acc=0;
  }
}*/
uint32_t filter1=0,filter2=0,filter3=0,filter4=0;uint16_t adc_value1=0,adc_value2=0,adc_value3=0,adc_value4=0;
uint16_t filterstatus1=0,filterstatus2=0,filterstatus3=0,filterstatus4=0;
void extra_gpio11(uint16_t u16Count)
{

  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x00;
  u8Command[2] = 0x00;
  filter1 += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    adc_value1=((filter1)/u16Count);
     
     if(adc_value1>1500){filterstatus1=1;}
  else {filterstatus1=0;}
  filter1=0;
  }
  
 
}
void extra_gpio12(uint16_t u16Count)
{

  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x40;
  u8Command[2] = 0x00;
  filter2 += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    adc_value2=((filter2)/u16Count);
     if(adc_value2>1500){filterstatus2=1;}
   else {filterstatus2=0;}
    
    filter2=0;
  }
  
}

void extra_gpio13(uint16_t u16Count)
{

  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0x80;
  u8Command[2] = 0x00;
  filter3 += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    adc_value3=((filter3)/u16Count);
     
     if(adc_value3>1500){filterstatus3=1;}
  else {filterstatus3=0;}
  filter3=0;
  }
  
 
}
void extra_gpio14(uint16_t u16Count)
{

  uint8_t u8Command[3] = {0};
  u8Command[0] = 0x06;
  u8Command[1] = 0xC0;
  u8Command[2] = 0x00;
  filter4 += fu16SendADCCommand(u8Command,3) & 0x0fff;
  if(u16Count >= NUMSAMPLES)
  {
    adc_value4=((filter4)/u16Count);
     if(adc_value4>1500){filterstatus4=1;}
   else {filterstatus4=0;}
    
    filter4=0;
  }
  
}
float Get_Water_In(void)
{
  return Temp_in;
}

float Get_Water_Out(void)
{
  return Temp_out;
}

float Get_Return_Air(void)
{
  return Temp_ret;
}

float Get_Flowrate(void)
{
  return Flow_lps;
}

float GetDuctPressure(void)
{
  return dp_measure_value;
}
float Get_Water_DP(void)
{
  return water_pres;
}
float Get_Sup_temp(void)
{
  return sup_tem;
}
uint16_t Get_gpio11(void)
{return gpiostatus7;}

uint16_t Get_gpio12(void)
{return gpiostatus8;}

uint16_t Get_gpio13(void)
{return gpiostatus9;}

uint16_t Get_gpio14(void)
{return gpiostatus10;}


uint16_t Get_gpio9(void)
{return filterstatus1;}

uint16_t Get_gpio7(void)
{return filterstatus2;}

uint16_t Get_gpio10(void)
{return filterstatus3;}

uint16_t Get_gpio8(void)
{return filterstatus4;}
