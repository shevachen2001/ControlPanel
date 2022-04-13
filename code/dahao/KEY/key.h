/*****************************************************************************
|  File Name: Key.h
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

#ifndef KEY_H
#define KEY_H
#include"EVENT\Event.h"
#include "Key\Key_Cfg.h"

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
#define SERVE_KEY_MAX                200
#define SERVE_KEY_PAGEMAX            10

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef enum 
{
    KEY_IDLE,
    KEY_PRESS_ONCE,
    KEY_PRESSING,
    KEY_WAIT_SHORT_RELEASE,
}Key_StateType;


typedef enum
{
    KEY_SCAN = 0,
    KEY_CMD_MAX
} Key_CmdType;

typedef struct
{
    Key_FlagType KeyType;
    uint16        Keydata;
} Key_ResultKeyType;

/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/
extern const Dr_OpertonType KeyDrive ;
extern void Key_Check(void);
/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/

#endif    


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
