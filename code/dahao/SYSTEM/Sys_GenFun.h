/*****************************************************************************
|  File Name: Sys_GenFun.h
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

#ifndef SYS_GENFUN_H
#define SYS_GENFUN_H
#include"PUBLIC\Std_Types.h"



/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32)((uint32)((Byte0) & 0x00FF) \
          + ((uint32)((Byte1) & 0x00FF) << 8) \
          + ((uint32)((Byte2) & 0x00FF) << 16) \
          + ((uint32)((Byte3) & 0x00FF) << 24)))

#define TWOHI_UINT32(a) (((a) >> 24) & 0xFF)
#define TWOLO_UINT32(a) (((a) >> 16) & 0xFF)
#define ONEHI_UINT32(a) (((a) >> 8) & 0xFF)
#define ONELO_UINT32(a) ((a) & 0xFF)

#define BUILD_UINT16(loByte, hiByte) \
          ((uint16)(((loByte) & 0x00FF) | (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
          ((uint8)(((loByte) & 0x0F) | (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((a) >> 4) & 0x0F)
#define LO_UINT8(a) ((a) & 0x0F)

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/


/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/

/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
extern    void Sys_GenFunDelayInit (void);
extern    void Sys_GenFunDelayUs (uint16 xus);
extern  void Sys_GenFunDelayMs (uint16 xms);
extern     uint8 Sys_GenFunChecksum(uint8 *pData);
extern uint32 Sys_GenFun8To32(uint8 *pData);
extern uint16 Sys_GenFun8To16(uint8 *pData);
extern void Sys_GenFun32To8(uint32 Data,uint8 *pData);
extern void Sys_GenFun16To8(uint16 Data,uint8 *pData);
extern uint8 Sys_GenFunLChecksum(uint16 Lenth,uint8 *pData);
extern uint32 Sys_GenFunhtonl(uint32 Data);
extern int16 Sys_GenFunitons(int16 Data);
extern uint16 Sys_GenFunhtons(uint16 Data);
extern float Sys_GenFunhtonf(float Data);

#endif    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




