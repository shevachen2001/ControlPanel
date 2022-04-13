/*****************************************************************************
|  File Name: EKEY.h
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
| 2013-10      LXW     Initial version 
|****************************************************************************/

#ifndef RTC_H_
#define RTC_H_
#include"PUBLIC\Std_Types.h"

/******************************************************************************
**********************Macro definitiont*******************************************
******************************************************************************/
#define     RTC_ADD                    0x60
#define     RTC_COMM_ALL_REALTIME    0x04
#define     RTC_COMM_TIME_REALTIME    0x06
#define     RTC_COMM_STATUS_REG1    0
#define     RTC_COMM_STATUS_REG2    0x02
#define     RTC_COMM_INT1            0x08
#define     RTC_COMM_INT2            0x0A


//#define      RTC_CHECK                STD_TRUE

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/

typedef uint8 Rtc_CommType;
#define RTC_FAILED                    I2C_SOFT_FAILED
#define RTC_SUCCESS                    I2C_SOFT_SUCCESS

typedef enum 
{
    RTC_CLOCKSET         = 0,    
    RTC_WLARMSET ,    
    SETRTCCONTROLMAX    
} Rtc_CmdType;

typedef struct {
    uint8    Year;
    uint8    Month;
    uint8    DayOfMonth;
    uint8    Hour;
    uint8    Minute;
    uint8    Second;
    uint8     DayOfWeek;
}Rtc_Type;

typedef struct {
    uint8    Year;
    uint8    Month;
    uint8    DayOfMonth;
}Rtc_CaledarType;

typedef struct {
    uint8    Hour;
    uint8    Minute;
    uint8    Second;
}Rtc_TimeType;
typedef struct {
    uint8    Flag;
    uint8    Hour;
    uint8    Minute;
    uint8    Second;
}Rtc_WlarmType;

/******************************************************************************
**********************Variable definition***************************************
******************************************************************************/
extern    uint8 Rtc_Wlarm_Enable;
extern const Dr_OpertonType RtcDrive;

extern Std_ReturnType Rtc_Ioctl(uint8 Cmd,void *pData);
extern Std_ReturnType Rtc_Read(void *pData);
extern void Rtc_Open(void *pData);
extern uint8 Main_GetWeekFromDay(unsigned char year,unsigned char month,unsigned char day) ;

/******************************************************************************
**********************Function declared****************************************
******************************************************************************/
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


