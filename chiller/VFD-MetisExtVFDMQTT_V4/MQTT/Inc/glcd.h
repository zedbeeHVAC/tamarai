/******************************************************************************
Copyright (c) 2019 released Swadha Energies Pvt. Ltd.  All rights reserved.
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

#define DISP_STR_HOR 0X01  //display string
#define UNIT_HOR 97
#define CURRENT_HOR 105
#define VALUE_HOR 65

#define RT_STR_VER 0XB1

#define IT_STR_VER 0XB3

#define OT_STR_VER 0XB5

#define FLOW_STR_VER 0XB7

#define CO2_STR_VER 0XB1

#define HUM_STR_VER 0XB3

#define POW_STR_VER 0XB5

#define ERR_STR_VER 0XB7
#define SUPAIR_STR_VER 0xB7


#define KWH_POS_HOR 78
#define KWH_POS_VER 0XB1

#define BTU_POS_HOR 30
#define BTU_POS_VER 0XB1

#define HZ_POS_HOR 91
#define HZ_POS_VER 0XB4

#define FREQ_POS_HOR 30
#define FREQ_POS_VER 0XB3

#define VFD_POS_HOR 40
#define VFD_POS_VER 0XB5

#define ONOFF_POS_HOR 60
#define ONOFF_POS_VER 0XB5

#define VOLT_POS_HOR 38
#define VOLT_POS_VER 0XB7

#define VOLTVAL_POS_HOR 1
#define VOLTVAL_POS_VER 0XB7


#define AMP_POS_HOR 118
#define AMP_POS_VER 0XB7

#define AMPVAL_POS_HOR 88
#define AMPVAL_POS_VER 0XB7

#define METIS_VER_HOR 0
#define METIS_VER_VER 0xB1
#define METIS_VALUE_HOR 38

#define SEN_CARD_HOR 0
#define SEN_CARD_VER 0xB3
#define SEN_CARD_VER_VALUE_HOR 50

#define ACT_VER_HOR 0
#define ACT_VER_VER 0xB5
#define ACT_VER_VLALUE_HOR 25

#define THERM_VER_HOR 0
#define THERM_VER_VER 0xB7
#define THERM_VALUE_VER_HOR 38

#define FRESH_POS_HOR 0
#define FRESH_POS_VER 0xB5
#define FRESH_POS_VALUE 72

#define PM1p0_POS_HOR 0
#define PM1p0_POS_VER 0xB5
#define PM1p0_POS_VALUE 66
#define PM10_POS_HOR 0
#define PM10_POS_VER 0xB7
#define PM10_POS_VALUE 66
#define PM2p5_POS_HOR 0
#define PM2p5_POS_VER  0xB7
#define PM2p5_POS_VALUE 72

#define VOC_POS_HOR 0
#define VOC_POS_VER 0xB2
#define VOC_POS_VALUE 60

#define UUID_VER_HOR 0
#define UUID_VER_VER 0xB1
#define UUID_VALUE_HOR 30

#define UVS_VER_HOR 0
#define UVS_VER_VER 0xB2
#define UVS_VALUE_HOR 60

#define UVR_VER_HOR 0
#define UVR_VER_VER 0xB3
#define UVR_VALUE_HOR 60

#define DISP_UVE_HOR 0X01  //display string
#define UV_LS_VER 0XB6
#define UV_ERR__VER 0XB5
#define UV_CUR__VER 0XB4
#define UVE_VALUE_HOR 60

#define IP_VER_HOR 0
#define IP_VER_VER 0xB3
#define IP_VALUE_HOR 18
#define SERVER_VER_HOR 0
#define SERVER_VER_VER 0xB2
#define SERVER_VALUE_HOR 24
#define SUB_VER_HOR 0
#define SUB_VER_VER 0xB5
#define SUB_VALUE_HOR 24

#define GAT_VER_HOR 0
#define GAT_VER_VER 0xB7
#define GAT_VALUE_HOR 24
enum EnError
{
    NO_ERROR,
    UNDERVOLTAGE_ERROR,  
    ALM1_ERROR,  // over current
    ALM2_ERROR,   // under voltage
    ALM3_ERROR,   // over temprature
    TEMP_IN_ERROR,
    TEMP_OUT_ERROR,
    FLOW_ERROR
};


struct TstDisplayParam
{
     uint16_t u16DisplayState;
     uint16_t u16DisplayPrevState;
};

struct TstMenuParam
{
     uint16_t u16DisplayState;
     uint16_t u16DisplayPrevState;
};

void
lcd_init(void);

void
display_logo(void);

void
lcd_display_image( char *Adata,uint32_t row_value,uint32_t Column,
                  uint32_t startrow,uint32_t startcolumn );

void
lcd_set_contrast(uint8_t val);

void
twopage_float(float fData,uint8_t u8Page,uint8_t u8Column);

void
lcd_print_float(float fData,uint8_t u8Page,uint8_t u8Column,uint8_t u8Font,
                 uint8_t u8Length,uint8_t selection);

void
lcd_print_char(uint8_t chars,uint8_t u8Page,uint8_t u8Column,
                uint8_t u8Font,uint8_t selection);

void
twopage_float(float fData,uint8_t u8Page,uint8_t u8Column);

void
LCD_PAGE_COL_CHECK_FONT1(void);

void
LCD_display_text(unsigned char *text, uint8_t startcol,uint8_t startpage);

void
LCD_PRINT_FONT1(unsigned char data,unsigned char selection);

void
LCD_PAGE_CLEAR(uint16_t page,uint16_t column);

void
print_page2_text(unsigned char text, uint8_t startcol,uint8_t startpage);

void
lcd_clear(void);

void
Display(unsigned char Font,unsigned char Page_No, unsigned char Col_No,
        unsigned char *MESSAGE , unsigned char Selection);

void
lcd_print_long(uint32_t data,uint8_t u8Length,uint8_t u8Page,uint8_t u8Column,
                uint8_t u8font,uint8_t selection);

void
fvMenuParamWrite(struct TstMenuParam *MenuParam_data);

void
fvMenuParamRead(struct TstMenuParam *MenuParam_data);

void
fvDisplayParamWrite(struct TstDisplayParam *DisplayParam_data);

void
fvDisplayParamRead(struct TstDisplayParam *DisplayParam_data);

void fvglcdMenuCommonHandler(void);

void fvMenuPageInc(void);

void fvMenuPageDec(void);
void lcd_refresh();
void lcd_print_double(float fData,uint8_t u8Page,uint8_t u8Column,uint8_t u8Font,
                 uint8_t u8Length,uint8_t selection);

void
lcd_print_string_position(uint8_t *str,uint8_t u8Page,uint8_t u8Column,
               uint8_t position,uint8_t u8Font);
uint16_t fu16GetDisplayState(void);