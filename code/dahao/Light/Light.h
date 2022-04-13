/*****************************************************************************
|  File Name: Light.h
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

#ifndef LIGHT_H
#define LIGHT_H
#include "PUBLIC\Std_Types.h"
#if (LIGHT_EN == STD_TRUE)

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
//#define LIGHT_CHECK        STD_TRUE


/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/    
typedef enum
{
     LIGHT_LOW = 0,
    LIGHT_STOP,
    LIGHT_OK,
    LIGHT_NG,
    LIGHT_KEY_LIGHT,
    LIGHT_NAME_MAX
} Light_NameType;

typedef enum
{
    LIGHT_IDLE = 0,
    LIGHT_NORMAL,
    LIGHT_VERY_FAST,
    LIGHT_FAST,
    LIGHT_SLOW,
    LIGHT_VERY_SLOW,
    
    LIGHT_OPEN,
    LIGHT_T_ERR,// 超时报错
    LIGHT_S_ERR,//查询报错
    LIGHT_MODE_MAX
} Light_StateType;


typedef struct {
    uint16 Hc595dat;
    Light_NameType lightname;
    Light_StateType state;
    uint8 wkTimes;
}Light_ComType;    


typedef struct {
    Light_StateType state;
    uint8 wkTimes;
    uint8 lightfg;
    uint8 lighttime;
    uint16 Hc595dat;
}Light_BkComType;

typedef struct {
    Light_StateType state;
    uint8         on_Timesx64ms;
    uint8         off_Timesx64ms;
}Light_WkParaType;

typedef struct {
    void (*lighton)(void);
    void (*lightoff)(void);
    void (*lighttogg)(void);
}Light_CntfunType;

typedef enum
{
    LIGHT_CMD_SCAN = 0,
    LIGHT_CMD_GET_STATE,
    LIGHT_CMD_MAX
} Light_CmdType;

/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/



extern const Dr_OpertonType LightDrive ;
/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
#if (LIGHT_CHECK == STD_TRUE)
extern void Light_Check(void);
#endif
#endif    

#endif    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


