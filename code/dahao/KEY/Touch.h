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

#ifndef TOUCH_H
#define TOUCH_H
#include"EVENT\Event.h"
#include "Key\Key_Cfg.h"

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
#define  TOUCH_KEY_BUF_MAX  30

#define  TOUCH_TIME_OUT_DATA   6000/64
#define  TOUCH_LOCK_TIME_OUT_DATA   30000/64
#define  TOUCH_TIME_LIGHT_OUT   128/64

typedef struct
{
   unsigned char FT_KeyFlag : 1; 
   unsigned char FT_FobFlag : 1; 
   unsigned char FT_MotorFlag : 1;
   unsigned char FT_KeyFinished:1; 
   unsigned char FT_MotorON_Finished:1; 
   unsigned char FT_MotorOF_Finished:1;   
   unsigned char FT_Bit7:1; 
   unsigned char FT_Bit8:1;
}FactoryTest_Bit8FlagType;

typedef struct
{
   unsigned char FT_KeyPressTimeout; 
   unsigned char FT_FobCheckTimeout; 
   unsigned char FT_MotorActionTimeout;
   unsigned char FT_AulockTimeout;
}FactoryTest_ActionType;

typedef struct
{
  unsigned char FactoryTestKeyIndex[6];
  FactoryTest_Bit8FlagType testFlag; 
  FactoryTest_ActionType   testTimeout;
}FactoryTestType;

extern FactoryTestType FactoryAppTest;






enum{
    TOUCH_STATE_IDLE,
    TOUCH_STATE_INPUT,
    TOUCH_STATE_ERROR,
};
/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/



/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/
extern const uint16 TouchLightStable[];
extern Std_Bit8Byte TouchLightFlag;  
extern uint8 TouchTimer64ms; 
extern uint16 TouchRstTimes;
/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
extern void Touch_Init(void);
extern void Touch_Proc(void);
extern void Touch_TimerProc(void);
extern void Touch_KeyProc(uint8* pTouchKey,uint8 lenth);
#endif    


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
