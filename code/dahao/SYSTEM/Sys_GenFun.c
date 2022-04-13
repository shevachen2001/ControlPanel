/*****************************************************************************
|  File Name: Sys_GenFun.c
|
|  Description: 
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
| Copyright (c) 2002-2012 by DAHAO TECHNOLOGY Co., Ltd. All rights reserved.
|
|               This software is copyright protected and proprietary 
|               to DAHAO TECHNOLOGY Co., Ltd. DAHAO TECHNOLOGY Co., Ltd 
|               grants to you only those rights as set out in the 
|               license conditions. All other rights remain with 
|               DAHAO TECHNOLOGY Co., Ltd.
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               DAHAO TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-10 1.00    lxw     Initial version 
|****************************************************************************/

#include"SYSTEM\Sys_GenFun.h"

/****************************************************************************************************
**Function:
  void Sys_GenFunDelayUs (void) 
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_GenFunDelayInit (void)
{  
//    memset(TIM2,0,sizeof(TIM2));
//    TIM2->PSCR = TIM2_PRESCALER_1;
}
/****************************************************************************************************
**Function:
  void Sys_GenFunDelayUs (void) 
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_GenFunDelayUs (uint16 xus)
{  
    do
    {
#ifdef  RC72MHz    
        NOP();    NOP(); 
        NOP();    NOP(); 
        NOP();    NOP();
        NOP();    NOP();
        NOP();    NOP();
        NOP();    
        NOP();    NOP(); 
        NOP();    NOP(); 
        NOP();    NOP();
        NOP();    NOP();
        NOP();    NOP();
        NOP();    
        NOP();    NOP(); 
        NOP();    NOP(); 
        NOP();    NOP();
        NOP();    NOP();
        NOP();    NOP();
        NOP();    
        NOP();    NOP(); 
        NOP();    NOP(); 
        NOP();    NOP();
        NOP();    NOP();
        NOP();    NOP();
        NOP();    
        NOP();    NOP(); 
        NOP();    NOP(); 
        NOP();    NOP();
        NOP();    NOP();
        NOP();    NOP();
        NOP();    
        NOP();    NOP(); 
        NOP();    NOP(); 
#endif        
        NOP();    NOP();
        NOP();    NOP(); 
        NOP();    
        NOP();    
        
    }
    while(--xus != 0);
}
/****************************************************************************************************
**Function:
  void Sys_GenFunDelayMs (void) 
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_GenFunDelayMs (uint16 xms)
{    
    do
    {
        Sys_GenFunDelayUs(1000);
#if (defined WATCHDOG_EN) && (WATCHDOG_EN == STD_TRUE)
//        WatchDogDrive.write(NULL);
#endif
    }
    while(--xms != 0);
}
/****************************************************************************************************
**Function:
  uint8 Sys_GenFunChecksum(uint8 *pData)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 Sys_GenFunChecksum(uint8 *pData)
{     
    uint8 length;
    uint8 checksum = 0;
    
    checksum = 0;
    length = *pData;
    for(; length > 0; length--)
    {
        checksum ^= *pData++;
    }
    return(checksum);
}
/****************************************************************************************************
**Function:
  uint8 Sys_GenFunLChecksum(uint16 Lenth,uint8 *pData)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint8 Sys_GenFunLChecksum(uint16 Lenth,uint8 *pData)
{     
    uint8 checksum = 0;
    
    checksum = 0;
    for(; Lenth > 0; Lenth--)
    {
        checksum ^= *pData++;
    }
    return(checksum);
}

/****************************************************************************************************
**Function:
  uint32 Sys_GenFun8To32(uint8 *pData)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint32 Sys_GenFun8To32(uint8 *pData)
{
    uint32 tempdata;
    
    tempdata = *pData;
    tempdata <<= 8;
    pData++;
    tempdata |= *pData;
    tempdata <<= 8;
    pData++;
    tempdata |= *pData;
    tempdata <<= 8;
    pData++;
    tempdata |= *pData;
    return tempdata;
}
/****************************************************************************************************
**Function:
  uint16 Sys_GenFun8To16(uint8 *pData)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint16 Sys_GenFun8To16(uint8 *pData)
{
    uint32 tempdata;
    
    tempdata = *pData;
    tempdata <<= 8;
    pData++;
    tempdata |= *pData;
    return tempdata;
}
/****************************************************************************************************
**Function:
  void Sys_GenFun16To8(uint16 Data,uint8 *pData)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_GenFun16To8(uint16 Data,uint8 *pData)
{
    *pData =(Data & 0x0000ff00)>>8;
    pData++;
    *pData = Data&0xff;
}
/****************************************************************************************************
**Function:
  void Sys_GenFun8To32(uint8 *pData)
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Sys_GenFun32To8(uint32 Data,uint8 *pData)
{
    *pData =(Data & 0xff000000)>>24;
    pData++;
    *pData =(Data & 0x00ff0000)>>16;
    pData++;
    *pData =(Data & 0x0000ff00)>>8;
    pData++;
    *pData = Data&0xff;
}
/****************************************************************************************************
**Function:
    uint16 Sys_GenFunhtons(uint16 n)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint16 Sys_GenFunhtons(uint16 Data)
{
    char nTmp;
    nTmp = ((char*)&Data)[0];
    ((char*)&Data)[0] = ((char*)&Data)[1];
    ((char*)&Data)[1] = nTmp;

    return Data;
}
/****************************************************************************************************
**Function:
    int16 Sys_GenFunitons(int16 Data)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
int16 Sys_GenFunitons(int16 Data)
{
    char nTmp;
    nTmp = ((char*)&Data)[0];
    ((char*)&Data)[0] = ((char*)&Data)[1];
    ((char*)&Data)[1] = nTmp;

    return Data;
}
/****************************************************************************************************
**Function:
    uint32 Sys_GenFunhtonl(uint32 n)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
uint32 Sys_GenFunhtonl(uint32 Data)
{
    char nTmp;

    nTmp = ((char*)&Data)[0];
    ((char*)&Data)[0] = ((char*)&Data)[3];
    ((char*)&Data)[3] = nTmp;

    nTmp = ((char*)&Data)[1];
    ((char*)&Data)[1] = ((char*)&Data)[2];
    ((char*)&Data)[2] = nTmp;    

    return Data;
}

/****************************************************************************************************
**Function:
    uint32 Sys_GenFunhtonl(uint32 n)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
float Sys_GenFunhtonf(float Data)
{
    char nTmp;

    nTmp = ((char*)&Data)[0];
    ((char*)&Data)[0] = ((char*)&Data)[3];
    ((char*)&Data)[3] = nTmp;

    nTmp = ((char*)&Data)[1];
    ((char*)&Data)[1] = ((char*)&Data)[2];
    ((char*)&Data)[2] = nTmp;    

    return Data;
}

