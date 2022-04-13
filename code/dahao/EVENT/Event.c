/*****************************************************************************
|  File Name: can_bsp.c
|
|  Description: 
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
| Copyright (c) 2002-2012 by DaHao TECHNOLOGY Co., Ltd. All rights reserved.
|
|               This software is copyright protected and proprietary 
|               to DaHao TECHNOLOGY Co., Ltd. DaHao TECHNOLOGY Co., Ltd 
|               grants to you only those rights as set out in the 
|               license conditions. All other rights remain with 
|               DaHao TECHNOLOGY Co., Ltd.
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               DaHao TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-10 1.00    lxw     Initial version 
|****************************************************************************/

/**************************************************************************************************/ 
/* Include file                                                                                   */                                                 
/**************************************************************************************************/

#include"..\EVENT\Event.h"

static Eve_Type Eve_Buff;
/****************************************************************************************************
**Function:
    void Event_Init(void)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Event_Init(void)
{
    Eve_Buff = 0;
}
/****************************************************************************************************
**Function:
    void Event_Set(Eve_IndexType Event)
**Author: rory
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Event_Set(Eve_IndexType Data)
{
    Eve_Buff |= Data;
}
/****************************************************************************************************
**Function:
    Eve_ReultType Event_Get(Eve_IndexType Event)
**Author: rory
**Description:
**Input: 
**Output:  1: has; 0:no
****************************************************************************************************/
Eve_ReultType Event_Get(Eve_IndexType Data)
{
    if(Eve_Buff & Data)
    {
        return EVE_OK;
    }
    else
    {
        return EVE_NO;
    }
}
/****************************************************************************************************
**Function: 
    void Event_Remove(Eve_IndexType Event)
**Author: rory
**Description:
**Input: 
    
**Output: 
****************************************************************************************************/
void Event_Remove(Eve_IndexType Data)
{
    Eve_Buff &= ~Data;
}
/****************************************************************************************************
**Function: 
    void Event_RemoveAll(void)
**Author: rory
**Description:
**Input: 
    
**Output: 
****************************************************************************************************/
void Event_RemoveAll(void)
{
    Eve_Buff = 0;
}

