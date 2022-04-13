
/*****************************************************************************
|  File Name: Sys_TimeBase.h
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

#ifndef SYS_TIMEBASE_H_
#define SYS_TIMEBASE_H_
#include"PUBLIC\Std_Types.h"

/******************************************************************************
**********************Macro definitiont*******************************************
******************************************************************************/
#define     SYS_TIMEBASE_64MS        Sys_TimeBaseFlag.bits.bit0
#define     SYS_TIMEBASE_128MS        Sys_TimeBaseFlag.bits.bit1
#define     SYS_TIMEBASE_256MS        Sys_TimeBaseFlag.bits.bit2
#define     SYS_TIMEBASE_512MS        Sys_TimeBaseFlag.bits.bit3
#define     SYS_TIMEBASE_1024MS        Sys_TimeBaseFlag.bits.bit4
#define     SYS_TIMEBASE_2048MS        Sys_TimeBaseFlag.bits.bit5
#define     SYS_TIMEBASE_4096MS        Sys_TimeBaseFlag.bits.bit6
#define     SYS_TIMEBASE_9012MS        Sys_TimeBaseFlag.bits.bit7

//#define     SYS_TIMEBASE_SYSBASE_32MS        Sys_TimeBaseSys32msFlag.bits.bit0
#define      SYS_TIMEBASE_SYSBASE_64MS        Sys_TimeBaseSys32msFlag.bits.bit1
#define     SYS_TIMEBASE_SYSBASE_128MS        Sys_TimeBaseSys32msFlag.bits.bit2
#define     SYS_TIMEBASE_SYSBASE_256MS        Sys_TimeBaseSys32msFlag.bits.bit3
#define      SYS_TIMEBASE_SYSBASE_512MS        Sys_TimeBaseSys32msFlag.bits.bit4
#define        SYS_TIMEBASE_SYSBASE_1024MS        Sys_TimeBaseSys32msFlag.bits.bit5
//#define     SYS_TIMEBASE_SYSBASE_32MS        Sys_TimeBaseSys2msFlag.bits.bit6
//#define     SYS_TIMEBASE_SYSBASE_64MS        Sys_TimeBaseSys2msFlag.bits.bit7



/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/


/******************************************************************************
**********************Variable definition***************************************
******************************************************************************/
extern    Std_Bit8Byte        Sys_TimeBaseSys32msFlag;  


/******************************************************************************
**********************Function declared****************************************
******************************************************************************/
extern void Sys_TimeBaseX64ms(void);
extern void Sys_TimeBaseX64msTask(void);
extern void Sys_TimeBaseX64msProc(void);
extern void Sys_TimeBaseInit (void); 

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

