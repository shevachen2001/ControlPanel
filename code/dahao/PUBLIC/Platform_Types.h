/*****************************************************************************
|  File Name: Usart_bsp.h
|
|  Description: Header File defined
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2012-09      LXW     Initial version 
|****************************************************************************/
/* Check if information is already included */
#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H


#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"
#include "Config.h"


/****************************************************************************/
/* macros                                                                   */
/****************************************************************************/

#define EI()  __enable_irq()//__asm("ei")
#define    DI()  __disable_irq()//__asm("di")
#define    NOP()  __asm("nop")
#define    HALT()  __asm("halt")

/****************************************************************************/
/* data types                                                               */
/****************************************************************************/
typedef     signed    char         int8;        
typedef     unsigned    short        uint16;     
typedef     short                       int16;        
typedef     unsigned    long        uint32; 
typedef     long                        int32;        
typedef     float                        fp32;        
typedef     double                      fp64;    

typedef     unsigned char            uint8;
typedef     unsigned char            BYTE;


/****************************************************************************/
/* data types                                                               */
/****************************************************************************/

/****************************************************************************/
/* IO Ddfeine                                                             */
/****************************************************************************/

/****************************************************************************/
/* RF                                               */
/****************************************************************************/

/****************************************************************************/
/* ADC                                              */
/****************************************************************************/
//#define ADC_PIN             3

#define BeepIO         5
#define Beep_OUT_1()   nrf_gpio_pin_set(BeepIO)
#define Beep_OUT_0()   nrf_gpio_pin_clear(BeepIO)

#define ReLay_PIN             1
#define ReLay_PIN_output()   nrf_gpio_cfg_output(ReLay_PIN) 
#define ReLay_PIN_High()     nrf_gpio_pin_set(ReLay_PIN) 
#define ReLay_PIN_Low()      nrf_gpio_pin_clear(ReLay_PIN) 



#define I2C_SOFT_CLK_PIN            17
#define I2c_SoftClockSetOutput()   nrf_gpio_cfg_output(I2C_SOFT_CLK_PIN) 
#define I2c_SoftClockSetInput()    nrf_gpio_cfg_input(I2C_SOFT_CLK_PIN,NRF_GPIO_PIN_NOPULL)

                                 
#define I2c_SoftClockGetData()      nrf_gpio_pin_read(I2C_SOFT_CLK_PIN)
#define I2c_SoftClockLow()          nrf_gpio_pin_clear(I2C_SOFT_CLK_PIN) 
#define I2c_SoftClockhigh()         nrf_gpio_pin_set(I2C_SOFT_CLK_PIN) 

#define I2C_SOFT_DAT_PIN            18
#define I2c_SoftDatSetOutput()       nrf_gpio_cfg_output(I2C_SOFT_DAT_PIN) 
#define I2c_SoftDatSetInput()          nrf_gpio_cfg_input(I2C_SOFT_DAT_PIN,NRF_GPIO_PIN_NOPULL) 

#define I2c_SoftDatGetData()        nrf_gpio_pin_read(I2C_SOFT_DAT_PIN)
#define I2c_SoftDatLow()               nrf_gpio_pin_clear(I2C_SOFT_DAT_PIN)
#define I2c_SoftDathigh()              nrf_gpio_pin_set(I2C_SOFT_DAT_PIN) 


#define BEEP_PIN       24
#define Beep_OutPutHigh()        nrf_gpio_pin_set(BEEP_PIN) 
#define Beep_OutPutLow()         nrf_gpio_pin_clear(BEEP_PIN)


#define TOUCH_SET_RST_PIN       16
#define Touch_SetRstLow()       nrf_gpio_pin_clear(TOUCH_SET_RST_PIN)
#define Touch_SetRstHigh()      nrf_gpio_pin_set(TOUCH_SET_RST_PIN) 


#define TOUCH_INTT_PIN          19
#define TouchIntatGetData()     nrf_gpio_pin_read(TOUCH_INTT_PIN)

#define TOUCH_I2CEN_PIN         20
#define Touch_I2cEnLow()        nrf_gpio_pin_clear(TOUCH_I2CEN_PIN)
#define Touch_I2cEnHigh()       nrf_gpio_pin_set(TOUCH_I2CEN_PIN) 


#define WIEGAND_DATA0_PIN        22
#define WIEGAND_DATA0Low()      nrf_gpio_pin_clear(WIEGAND_DATA0_PIN)
#define WIEGAND_DATA0High()     nrf_gpio_pin_set(WIEGAND_DATA0_PIN) 


#define WIEGAND_DATA1_PIN        23
#define WIEGAND_DATA1Low()      nrf_gpio_pin_clear(WIEGAND_DATA1_PIN)
#define WIEGAND_DATA1High()     nrf_gpio_pin_set(WIEGAND_DATA1_PIN) 


/* RC522 RST  */
#define RC522_RST_PIN             15
#define RC522_SetRstHigh()         nrf_gpio_pin_set(RC522_RST_PIN)
#define RC522_SetRstLow()          nrf_gpio_pin_clear(RC522_RST_PIN)



/* RC522 CS  */
#define RC522_CS_PIN             13
#define RC522_SetCsHigh()       nrf_gpio_pin_set(RC522_CS_PIN)
#define RC522_SetCsLow()        nrf_gpio_pin_clear(RC522_CS_PIN)

#define FLASH_CS_PIN             14
#define Flash_SpiCsHigh()       nrf_gpio_pin_set(FLASH_CS_PIN)
#define Flash_SpiCsLow()        nrf_gpio_pin_clear(FLASH_CS_PIN)

/* MF522_MSCK  */
#define RC522_MSCK_PIN             12
#define RC522_ClkHigh()         NRF_GPIO->OUTSET = 0x1000//(1UL << RC522_MSCK_PIN);
#define RC522_ClkLow()          NRF_GPIO->OUTCLR = 0x1000//(1UL << RC522_MSCK_PIN);

/* MF522_MISO  */
#define RC522_MISO_PIN             10
#define RC522_GetMisoDat()      nrf_gpio_pin_read(RC522_MISO_PIN)
#define RC522_MisoHigh()        nrf_gpio_pin_set(RC522_MISO_PIN)
#define RC522_MisoLow()         nrf_gpio_pin_clear(RC522_MISO_PIN)

/* MF522_MOSI  */
#define RC522_MOSI_PIN             11
#define RC522_MosiHigh()       NRF_GPIO->OUTSET =0x800 //(1UL << RC522_MOSI_PIN);
#define RC522_MosiLow()        NRF_GPIO->OUTCLR = 0x800//(1UL << RC522_MOSI_PIN);

#define HC595_DAT                 RC522_MISO_PIN
#define Hc595_Dat_High()       nrf_gpio_pin_set(HC595_DAT)
#define Hc595_Dat_Low()        nrf_gpio_pin_clear(HC595_DAT)

#define HC595_SCLK                 RC522_MSCK_PIN
#define Hc595_Sclk_High()       nrf_gpio_pin_set(HC595_SCLK)
#define Hc595_Sclk_Low()        nrf_gpio_pin_clear(HC595_SCLK)

#define HC595_LCLK                 21
#define Hc595_Lclk_High()       nrf_gpio_pin_set(HC595_LCLK)
#define Hc595_Lclk_Low()        nrf_gpio_pin_clear(HC595_LCLK)

//#define HC595_POWER                 0
//#define Hc595_PowerOn()         nrf_gpio_pin_clear(HC595_POWER)
//#define Hc595_PowerOff()        nrf_gpio_pin_set(HC595_POWER)

#define VDD_POWER                 9//25
#define Vdd_PowerOn()           nrf_gpio_pin_set(VDD_POWER)

#define RTC_POWER                 0
#define RTC_PowerOn()         nrf_gpio_pin_set(RTC_POWER)
#define RTC_PowerOff()        nrf_gpio_pin_clear(RTC_POWER)

#define TSM_POWER                2
#define TSM_PowerOn()          nrf_gpio_pin_set(TSM_POWER)
#define TSM_PowerOff()         nrf_gpio_pin_clear(TSM_POWER)

#define Infrared_POWER            7
#define Infrared_PowerOn()     nrf_gpio_pin_set(Infrared_POWER)

#define A485_POWER                6
#define A485_PowerOn()          nrf_gpio_pin_set(A485_POWER)
#define A485_PowerOff()         nrf_gpio_pin_clear(A485_POWER)

#define WG_POWER                3
#define WG_PowerOn()          nrf_gpio_pin_set(WG_POWER)
#define WG_PowerOff()         nrf_gpio_pin_clear(WG_POWER)


#define CFG_KEY_PIN             30
#define CfgKeyGetData()         nrf_gpio_pin_read(CFG_KEY_PIN)

#define RELAY_SW_PIN            4
#define RELAYSWGetData()        nrf_gpio_pin_read(RELAY_SW_PIN)

#define Infrared_PIN             25
#define InfraredGetData()        nrf_gpio_pin_read(Infrared_PIN)

#define Test_PIN             3
#define IntestGet()          nrf_gpio_pin_read(Test_PIN)


#endif /* ifndef PLATFORM_TYPES_H */
/************************** end of file *************************************/
