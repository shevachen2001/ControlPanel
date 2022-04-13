
/*****************************************************************************
|  File Name: Sys_TimeBase.c
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
#include"SYSTEM\Sys_TimeBase.h"

/****************************************************************************************************
**Function:
  void TIM0_IRQHandler (void) 
**Author: lxw 
**Description:
**Input: 
**Output: 
****************************************************************************************************/
/*void SysTick_Handler(void)
{
    Sys_TimeBaseX2ms();
    Sys_TimeBaseX2msTask();
}*/


void TIMER2_IRQHandler()
{  
    if(NRF_TIMER2->EVENTS_COMPARE[0] == 1)
    {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;     //Çå³ýÊÂ¼þ
        Sys_TimeBaseX64ms();
        Sys_TimeBaseX64msTask();
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

