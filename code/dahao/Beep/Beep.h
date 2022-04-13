/*****************************************************************************
|  File Name: Beep.h
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

#ifndef BEEP_H
#define BEEP_H
#include"PUBLIC\Std_Types.h"
#if (BEEP_EN == STD_TRUE)

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
typedef enum
{
    BEEP_IDLE = 0,
    BEEP_NORMAL,
    BEEP_VERY_FAST,
    BEEP_FAST,
    BEEP_SLOW,
    BEEP_VERY_SLOW,
    BEEP_OPEN,
    BEEP_LOW,
    BEEP_AL1,
    BEEP_AL2,
    BEEP_AL3,
    BEEP_OPEN_ERR,
    
    BEEP_TEST1,
    BEEP_OTHER,
} Beep_StateType;


typedef struct {
    Beep_StateType state;
    uint8 wkTimes;
}Beep_ComType;    


typedef struct {
    Beep_StateType state;
    uint8 wkTimes;
    uint8 beepfg;
    uint8 beeptime;
}Beep_BkComType;
typedef struct {
    uint16         on_Timesx64ms;
    uint16         off_Timesx64ms;
    uint8         Duty;
    uint16         Freq;
}Beep_WksubParaType;



typedef struct {
    const Beep_StateType state;
    Beep_WksubParaType *WksubPara;
}Beep_WkParaType;
typedef enum
{
    BEEP_CMD_SCAN = 0,
    BEEP_CMD_GET_STATE,
    BEEP_CMD_MAX
} Beep_CmdType;

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef struct
{
    uint8     mode;
    uint8    times;
} Beep_ParmType;

#define BEEP_ERROR_PROMPT   1  /* 提示*/
#define BEEP_ERROR_TOUCH_PASWD   2  /* 系统密码和卡片错误*/
#define BEEP_ERROR_LOW_VOL        3  /* 低压报警*/
#define BEEP_ERROR_TIME_OUT   4  /* 超时*/
#define BEEP_ERROR_ID   5 /* 设备ID错误*/
#define BEEP_ERROR_USR_PASWD   6 /* 用户密码错误*/
#define BEEP_ERROR_FULL   7  /*  密码和卡片存储已满*/
#define BEEP_ERROR_EMERGENCY   8  /* 应急模式*/

/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/

extern const Dr_OpertonType BeepDrive ;
/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
#endif

#endif    


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

