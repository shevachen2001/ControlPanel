/*****************************************************************************
|  File Name: Key_Cfg.h
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

#ifndef KEY_CFG_H
#define KEY_CFG_H
#include"PUBLIC\Std_Types.h"


/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
#define   KEY_EVNET_NULL    0xff


#define     KEY_MAX_NUM        1

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef enum 
{
    KEY_NO_RESULT=0x00,
    KEY_0,
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_7,
    KEY_8,
    KEY_9,
    KEY_CLEAR, /* �Ǻ�*/
    KEY_ENTER, /* # */
    KEY_CFG,
    KEY_NULL = 0xff
}Key_DataType;

typedef enum 
{
    KEY_STATE_NULL = 0,
    KEY_SHORT = 1,
    KEY_LONG = 2,
}Key_FlagType;

#define KEY1_DATA    0x01
#define KEY2_DATA    0x02
#define KEY3_DATA    0x04
/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/


/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
//#undef EXTERN

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

